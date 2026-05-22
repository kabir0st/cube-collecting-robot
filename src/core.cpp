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
#include <servo.h>


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
                       const RansacParams& ransac) {
    hh::Lidar lidar;
    if (!start_lidar(lidar)) {
        return;
    }
    std::thread thread_loop_collect_frames(loop_collect_frames,
                std::ref(lidar), std::ref(lidar_data));

    // Wait for the circular buffer to fill. ~5 Hz frame rate × 12 s ≈ 60
    // frames, comfortably above FRAME_COUNT=50, so find_walls sees a
    // fully-populated buffer for RANSAC wall extraction.
    LOG("Main") << "Collecting lidar frames for map build...";
    std::this_thread::sleep_for(std::chrono::seconds(12));

    const double measured_noise = get_lidar_sensor_noise(lidar_data);
    lidar_data.sensor_noise.store(measured_noise);
    LOG("Main") << "Lidar Sensor Noise : " << measured_noise;

    std::vector<LineSeg> ref_map = find_walls(lidar_data, ransac);
    if (ref_map.empty()) {
        LOGE("Main") << "find_walls produced an empty reference map; "
                        "Cox has nothing to register against. Aborting.";
        app::shutdown.store(true);
        stop_lidar(lidar);
        thread_loop_collect_frames.join();
        return;
    }
    LOG("Main") << "Built reference map (" << ref_map.size() << " walls).";
    calib::compute_map_bounds(ref_map);
    telemetry::send_map(ref_map);

    // Seed the KF at the starting pose = home (0, 0, 0). If cox later
    // disagrees it will repin via the relocalize sweep — "I'm somewhere
    // unknown" is recoverable, not fatal.
    kf.reset(Pos2D(0.0, 0.0, 0.0), KalmanFilter::default_cov());

    std::thread thread_loop_scan_cox(
        loop_scan_cox_estimation, std::ref(ref_map),
        std::ref(lidar_data),
        std::ref(dash), std::ref(kf)
    );
    thread_loop_scan_cox.join();
    thread_loop_collect_frames.join();
    stop_lidar(lidar);
}


void loop_control(Dashboard& dash,
                  KalmanFilter& kf, TrackerResult& target,
                  LidarData& lidar_data,
                  const MissionConfig& mission) {
    // sequence (fwd 400 mm -> +90° + 300 mm -> back home)
    constexpr bool IS_TEST = false;

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

    // Home is fixed at the localization origin (0, 0, 0).
    sm.configure(0.0, 0.0, 0.0,
                 mission.home_margin, mission.arm_length_mm);

    // Block any mission entry (TEST, SEARCH, or future GOTO seeded from
    // here) until cox has produced a fix that the KF actually accepted.
    // Otherwise the SM kicks off on the seeded (0,0,0) pose and the robot
    // drives before localization is real. The 15 s timeout exists so a
    // misconfigured map / bad scan doesn't hang forever — we fall through
    // and drive on dead reckoning, matching the pre-gate behaviour.
    {
        constexpr auto WAIT_TIMEOUT = std::chrono::seconds(15);
        const auto t_start = std::chrono::steady_clock::now();
        while (!app::cox_first_fix.load(std::memory_order_acquire)
            && !app::shutdown.load()) {
            if (std::chrono::steady_clock::now() - t_start > WAIT_TIMEOUT) {
                LOGE("Main") << "Cox first-fix wait timed out after 15s — "
                                "starting on dead-reckoned pose.";
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (app::cox_first_fix.load(std::memory_order_acquire)) {
            LOG("Main") << "Cox first fix acquired — starting mission.";
        }
    }

    if (IS_TEST) {
        sm.start_test();
    } else {
        sm.start();
    }

    while(!app::shutdown) {
        // giving time for 2 readings
        std::this_thread::sleep_for(std::chrono::milliseconds(40));

        // Advance any pending gripper slew by one STEP_DEG (paced internally
        // to STEP_MS). Non-blocking: keeps the 5 V rail safe without
        // stalling odometry / motor updates.
        servo_tick();

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
        const StateMachine::Command cmd = sm.step(p, cov, dash);
        double v_cmd = 0.0, w_cmd = 0.0;
        switch (cmd.mode) {
            case StateMachine::Command::Mode::DIRECT:
                v_cmd = cmd.v_mm_s;
                w_cmd = cmd.omega_rad_s;
                break;
            case StateMachine::Command::Mode::PATH_FOLLOW:
                pp.compute(p, v_cmd, w_cmd);
                break;
            case StateMachine::Command::Mode::STOP:
            default:
                v_cmd = 0.0;
                w_cmd = 0.0;
                break;
        }
        // Hold motors while cox is running a relocalize sweep -- driving on
        // a stale pose is worse than standing still. Also hold on a forward-
        // arc obstacle: loop_collect_frames sets app::obstacle_ahead when
        // any beam inside ±OBSTACLE_ARC_HALF_DEG is closer than
        // OBSTACLE_DIST_MM. Reverse motion (v < 0) is allowed so the robot
        // can back out of a near-miss.
        if (app::relocalizing.load()) {
            v_cmd = 0.0;
            w_cmd = 0.0;
        } else if (app::obstacle_ahead.load() && v_cmd > 0.0) {
            v_cmd = 0.0;
            w_cmd = 0.0;
        }

        // motor.set_velocity(v_cmd, w_cmd);

        uint64_t rejects = 0;
        motor.get_spi_stats(rejects);
        dash.set("Motor", "spi_rej", rejects);
        dash.set("Motor", "v_cmd",  v_cmd);
        dash.set("Motor", "w_cmd_deg_s", w_cmd * 180.0 / M_PI);

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
            const Pos2D home_pose(0.0, 0.0, 0.0);
            telemetry::send_state(p, cov,
                                  home_pose,
                                  pp.get_active_target(),
                                  pp.has_active_target(),
                                  lidar_data);
        }
    }
    motor.stop();
}
