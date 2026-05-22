#include <ina219.h>
#include <core.hpp>
#include <global.hpp>
#include <types.hpp>
#include <thread>
#include <algorithm>
#include <cmath>
#include <cstdint>

#include <controller.hpp>
#include <loggers.hpp>
#include <Lidar.hpp>
#include <utils.hpp>
#include <dashboard.hpp>
#include <calibrations.hpp>
#include <cox.hpp>
#include <motor.hpp>
#include <odometry.hpp>
#include <pure_pursuit.hpp>
#include <kalman.hpp>
#include <telemetry.hpp>


void loop_check_battery(INA219& power_interface, Dashboard& dash) {
    int hits = 0;
    int absent_hits = 0;
    constexpr int ABSENT_HITS_CAP = 10;  // ~20 seconds at CHECK_PERIOD = 2s
    while (!app::shutdown) {
        float v = power_interface.supply_voltage();
        float mA = power_interface.current();
        dash.set("Battery", "Voltage", v);
        dash.set("Battery", "Current", mA);
        dash.set("Battery", "SoC %", lipo_soc_pct(v));
        if (!std::isfinite(v) || !std::isfinite(mA)) {
            LOGE("Battery") << "INA219 returned non-finite reading "
                            << "(v=" << v << " mA=" << mA
                            << ") — treating as sensor fault, shutting down.";
            app::shutdown.store(true);
            break;
        }
        if (v < BATTERY_PRESENT_MIN_V) {
            hits = 0;
            if (++absent_hits >= ABSENT_HITS_CAP) {
                LOGE("Battery") << "Rail below BATTERY_PRESENT_MIN_V for "
                                << absent_hits * CHECK_PERIOD
                                << " s — INA219 likely faulted. Shutting down.";
                app::shutdown.store(true);
                break;
            }
        } else if (v <= LOW_BATTERY_LIMIT) {
            absent_hits = 0;
            hits++;
        } else {
            absent_hits = 0;
            hits = 0;
        }
        if (hits >= LOW_HITS_NEEDED)
        {
            LOGE("Battery") << "LOW (~25%): " << v << " V (" << mA
                            << " mA) — please recharge soon";
            LOGE("Battery") << "!!! Shutting down for now. !!!";
            app::shutdown.store(true);
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

void loop_localization(Dashboard& dash,
                       LidarData& lidar_data, KalmanFilter& kf,
                       const ArenaConfig& arena, double home_theta) {
    // mapping;
    hh::Lidar lidar;
    if (!start_lidar(lidar)) {
        return;
    }
    // starting lidar data collection
    std::thread thread_loop_collect_frames(loop_collect_frames,
                std::ref(lidar), std::ref(lidar_data));

    std::this_thread::sleep_for(std::chrono::seconds(2));
    const double measured_noise = get_lidar_sensor_noise(lidar_data);
    lidar_data.sensor_noise.store(measured_noise);
    LOG("Main") << "Lidar Sensor Noise : " << measured_noise;

    std::vector<LineSeg> ref_map;
    if (arena.in_arena) {
        const std::vector<LineSeg> observed = find_walls(lidar_data,
                                                         arena.ransac_arena);
        const std::vector<LineSeg> expected = build_arena_map(
            arena.width_mm, arena.length_mm);
        if (validate_arena_walls(expected, observed,
                                 arena.validation_tol_mm,
                                 arena.validation_tol_rad)) {
            ref_map = expected;
        } else {
            ArenaFit fit;
            if (!try_fit_arena_fallback(observed, arena, fit)) {
                LOGE("Main") << "Arena validation failed -- refusing to start.";
                app::shutdown.store(true);
                stop_lidar(lidar);
                thread_loop_collect_frames.join();
                return;
            }
            LOG("MAP") << "Arena validation fell back to closest-short-wall"
                       << " home" << (fit.rotated ? " (rotated 90 deg)" : "")
                       << "; seeding theta=" << fit.home_theta;
            ref_map = std::move(fit.ref_map);
            home_theta = fit.home_theta;
        }
    } else {
        ref_map = find_walls(lidar_data, arena.ransac_room);
        if (ref_map.empty()) {
            LOGE("Main") << "find_walls produced an empty reference map; "
                            "Cox has nothing to register against. Aborting.";
            app::shutdown.store(true);
            stop_lidar(lidar);
            thread_loop_collect_frames.join();
            return;
        }
    }
    LOG("Main") << "Built reference map (" << ref_map.size() << " walls).";
    calib::compute_arena_bounds(ref_map);
    telemetry::send_map(ref_map);

    // Seed the KF at the expected home pose. If the robot was actually
    // placed elsewhere the seed is wrong, cox will return an untrusted
    // estimate on the first cycle, and loop_scan_cox_estimation's
    // relocalize sweep will find the real pose and re-pin the KF. No
    // abort -- "I'm somewhere unknown" is recoverable, not fatal.
    if (arena.in_arena) {
        kf.reset(Pos2D(0.0, 0.0, home_theta), KalmanFilter::default_cov());
    }

    // starting cox estimation
    std::thread thread_loop_scan_cox(
        loop_scan_cox_estimation, std::ref(ref_map),
        std::ref(lidar_data),
        std::ref(dash), std::ref(kf)
    );
    // joining threads
    thread_loop_scan_cox.join();
    thread_loop_collect_frames.join();
    stop_lidar(lidar);
}


void loop_control(Dashboard& dash,
                  KalmanFilter& kf, TrackerResult& target,
                  LidarData& lidar_data,
                  const MissionConfig& mission) {
    Motor motor;
    Odometry odom;
    PurePursuitController pp;
    StateMachine sm(target, pp);
    double total_d = 0;
    double total_a = 0;
    if (!motor.start())
    {
        LOGE("Main") << "Motor start failed";
        return;
    }
    {
        // resetting
        double l0_mm, r0_mm;
        motor.get_encoders(l0_mm, r0_mm);
        odom.reset(l0_mm, r0_mm);
    }

    // sm.configure(mission.home_x_mm, mission.home_y_mm, mission.home_theta,
    //              mission.home_margin, mission.arm_length_mm);
    // sm.start();

    while(!app::shutdown) {
        // giving time for 2 readings
        std::this_thread::sleep_for(std::chrono::milliseconds(40));

        double l_mm, r_mm;
        motor.get_encoders(l_mm, r_mm);
        dash.set("Motor", "L_mm", l_mm);
        dash.set("Motor", "R_mm", r_mm);

        double v_sm = 0.0, w_sm = 0.0;
        motor.get_current_v_w(v_sm, w_sm);
        dash.set("Motor", "v_mm_s", v_sm);
        dash.set("Motor", "w_deg_s", w_sm * 180.0 / M_PI);

        const OdomStep step = odom.update(l_mm, r_mm);
        if (step.valid) {
            total_d += step.d_mm;
            total_a += step.d_theta;
            dash.set("Odometry", "d_mm",  step.d_mm);
            dash.set("Odometry", "d_deg", step.d_theta * 180.0 / M_PI);
            dash.set("Odometry", "D_mm",  total_d);
            dash.set("Odometry", "A_deg", total_a * 180.0 / M_PI);
            dash.set("Odometry", "sD_mm", std::sqrt(step.var_d));
            dash.set("Odometry", "sA_deg",
                     std::sqrt(step.var_theta) * 180.0 / M_PI);
            kf.predict(step.d_mm, step.d_theta,
                       step.var_d, step.var_theta,
                       step.cov_d_theta);
        }

        Pos2D p;
        Pose2DCov cov;
        kf.snapshot(p, cov);
        // const StateMachine::Command cmd = sm.step(p, dash);
        // double v_cmd = 0.0, w_cmd = 0.0;
        // switch (cmd.mode) {
        //     case StateMachine::Command::Mode::DIRECT:
        //         v_cmd = cmd.v_mm_s;
        //         w_cmd = cmd.omega_rad_s;
        //         break;
        //     case StateMachine::Command::Mode::PATH_FOLLOW:
        //         pp.compute(p, v_cmd, w_cmd);
        //         break;
        //     case StateMachine::Command::Mode::STOP:
        //     default:
        //         v_cmd = 0.0;
        //         w_cmd = 0.0;
        //         break;
        // }
        // Hold motors while cox is running a relocalize sweep -- driving on
        // a stale pose is worse than standing still. Also hold on a forward-
        // arc obstacle: loop_collect_frames sets app::obstacle_ahead when
        // any beam inside ±OBSTACLE_ARC_HALF_DEG is closer than
        // OBSTACLE_DIST_MM. Reverse motion (v < 0) is allowed so the robot
        // can back out of a near-miss.
        // if (app::relocalizing.load()) {
        //     v_cmd = 0.0;
        //     w_cmd = 0.0;
        // } else if (app::obstacle_ahead.load() && v_cmd > 0.0) {
        //     v_cmd = 0.0;
        //     w_cmd = 0.0;
        // }
        // motor.set_velocity(v_cmd, w_cmd);

        uint64_t rejects = 0;
        motor.get_spi_stats(rejects);
        dash.set("Motor", "spi_rej", rejects);
        // dash.set("Motor", "v_cmd",  v_cmd);
        // dash.set("Motor", "w_cmd_deg_s", w_cmd * 180.0 / M_PI);

        // Fused pose + uncertainties (1σ).
        dash.set("KF", "X_mm",    p.x);
        dash.set("KF", "Y_mm",    p.y);
        dash.set("KF", "Th_deg",  p.theta * 180.0 / M_PI);
        dash.set("KF", "sX_mm",   std::sqrt(std::max(0.0, cov.xx)));
        dash.set("KF", "sY_mm",   std::sqrt(std::max(0.0, cov.yy)));
        dash.set("KF", "sTh_deg",
                 std::sqrt(std::max(0.0, cov.tt)) * 180.0 / M_PI);

        dash.set("Mission", "state",
                 std::string(StateMachine::state_name(sm.state())));

        // Gated telemetry: ~10 Hz regardless of the 25 Hz control loop.
        static auto last_send = std::chrono::steady_clock::now();
        const auto now = std::chrono::steady_clock::now();
        if (now - last_send >= std::chrono::milliseconds(100)) {
            last_send = now;
            const Pos2D home_pose(mission.home_x_mm,
                                  mission.home_y_mm,
                                  mission.home_theta);
            telemetry::send_state(p, cov,
                                  home_pose,
                                  pp.get_active_target(),
                                  pp.has_active_target(),
                                  lidar_data);
        }
    }
    motor.stop();
}
