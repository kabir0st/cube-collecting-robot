#include <ina219.h>
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
    lidar_data.sensor_noise = get_lidar_sensor_noise(lidar_data);
    LOG("Main") << "Lidar Sensor Noise : " << lidar_data.sensor_noise;

    std::vector<LineSeg> ref_map;
    if (arena.in_arena) {
        const std::vector<LineSeg> observed = find_walls(lidar_data);
        const std::vector<LineSeg> expected = build_arena_map(
            arena.width_mm, arena.length_mm);
        if (!validate_arena_walls(expected, observed,
                                  arena.validation_tol_mm,
                                  arena.validation_tol_rad)) {
            LOGE("Main") << "Arena validation failed -- refusing to start.";
            app::shutdown.store(true);
            stop_lidar(lidar);
            thread_loop_collect_frames.join();
            return;
        }
        ref_map = expected;
    } else {
        ref_map = find_walls(lidar_data);
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

    // Home sanity-check: only meaningful when we're operating against the
    // analytical map (otherwise the RANSAC map was just extracted from
    // wherever the robot is sitting; matching it against itself is
    // tautological). Run Cox once seeded at home; if the converged pose
    // drifts beyond tolerance the robot is somewhere else.
    if (arena.in_arena) {
        const Pos2D home_pose(0.0, 0.0, home_theta);
        kf.reset(home_pose, KalmanFilter::default_cov());
        CoxScanMatchAlgorithm probe(ref_map, home_pose, lidar_data);
        const PosEstimate sanity = probe.estimate(home_pose);
        const double drift_xy   = std::hypot(sanity.pos.x, sanity.pos.y);
        const double drift_theta = std::fabs(
            std::atan2(std::sin(sanity.pos.theta - home_theta),
                       std::cos(sanity.pos.theta - home_theta)));
        const bool ok = sanity.converged
                     && sanity.inliers >= 10
                     && drift_xy   <= arena.home_sanity_tol_mm
                     && drift_theta <= arena.home_sanity_tol_rad;
        if (!ok) {
            LOGE("Main") << "Home sanity-check failed: pose drift ("
                         << sanity.pos.x << ", " << sanity.pos.y << ", "
                         << sanity.pos.theta << " rad), inliers="
                         << sanity.inliers << ", converged="
                         << (sanity.converged ? "yes" : "no")
                         << " -- robot is not at home.";
            app::shutdown.store(true);
            stop_lidar(lidar);
            thread_loop_collect_frames.join();
            return;
        }
        // Re-pin to the exact home pose: we trust the placement, not the
        // single scan-match.
        kf.reset(home_pose, KalmanFilter::default_cov());
        LOG("Main") << "Home sanity-check OK (drift " << drift_xy << " mm, "
                    << (drift_theta * 180.0 / M_PI) << " deg).";
    }

    // starting cox estimation
    std::thread thread_loop_scan_cox(
        loop_scan_cox_estimation, std::ref(ref_map),
        std::ref(lidar_data),
        std::ref(dash), std::ref(kf),
        arena.kidnap_threshold,
        arena.relocalize_grid_mm,
        arena.relocalize_grid_rad
    );
    // joining threads
    thread_loop_scan_cox.join();
    thread_loop_collect_frames.join();
    stop_lidar(lidar);
}


void loop_control(Dashboard& dash,
                  KalmanFilter& kf, TrackerResult& target) {
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

    // GO-TO TEST: drive to a single (x, y, theta) world-frame pose via
    // spline + PurePursuit. The SEARCH/APPROACH/CARRY_HOME mission is *not*
    // started; we use the lab-mode go_to() entry directly, which is
    // orthogonal to the vision-driven mission and latches DONE on arrival.
    constexpr double GOTO_X     = 300.0;  // mm
    constexpr double GOTO_Y     = 0.0;    // mm
    constexpr double GOTO_THETA = 0.0;    // rad
    sm.go_to(GOTO_X, GOTO_Y, GOTO_THETA);

    while(!app::shutdown) {
        // giving time for 2 readings
        std::this_thread::sleep_for(std::chrono::milliseconds(40));

        double l_mm, r_mm;
        motor.get_encoders(l_mm, r_mm);
        dash.set("Motor", "L_mm", l_mm);
        dash.set("Motor", "R_mm", r_mm);

        double v_sm = 0.0, w_sm = 0.0;
        motor.get_current_v_w(v_sm, w_sm);
        dash.set("Motor", "v", v_sm);
        dash.set("Motor", "w", w_sm);

        const OdomStep step = odom.update(l_mm, r_mm);
        if (step.valid) {
            total_d += step.d_mm;
            total_a += step.d_theta;
            dash.set("Odometry", "d_dist", step.d_mm);
            dash.set("Odometry", "d_ang",  step.d_theta * 180.0 / M_PI);
            dash.set("Odometry", "Total D", total_d);
            dash.set("Odometry", "Total A", total_a * 180.0 / M_PI);
            kf.predict(step.d_mm, step.d_theta,
                       step.var_d, step.var_theta,
                       step.cov_d_theta);
        }

        Pos2D p;
        Pose2DCov cov;
        kf.snapshot(p, cov);
        const StateMachine::Command cmd = sm.step(p, dash);
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
        motor.set_velocity(v_cmd, w_cmd);

        uint64_t rejects = 0;
        motor.get_spi_stats(rejects);
        dash.set("Motor", "spi_rej", rejects);
        dash.set("KF", "X",     p.x/10);
        dash.set("KF", "Y",     p.y/10);
        dash.set("KF", "Theta", p.theta * 180.0 / M_PI);
    }
    motor.stop();
}
