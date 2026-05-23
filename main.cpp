#include <algorithm>
#include <cmath>
#include <cstdio>
#include <csignal>
#include <iostream>
#include <mutex>
#include <thread>

#include <calibrations.hpp>
#include <global.hpp>
#include <loggers.hpp>
#include <motor.hpp>
#include <types.hpp>

// TEMPORARY CALIBRATION BUILD — drives the chassis 300 mm forward (per
// encoders) and prints the per-wheel reported distance so the operator
// can compare against a tape-measured ground truth and adjust
// LEFT/RIGHT_TICKS_PER_MM in motor.hpp. Restore the mission main with:
//     git checkout main.cpp
// (or flip CALIBRATION_BUILD below to 0).
#define CALIBRATION_BUILD 1

// app:: atomics and ROBOT_SPEED are referenced from other compile units
// in src/, so they must stay defined here regardless of which main() is
// active.
std::atomic<bool> app::shutdown{false};
std::atomic<bool> app::lidar_ready{false};
std::atomic<bool> app::lidar_stop{false};
std::atomic<bool> app::obstacle_ahead{false};
std::atomic<bool> app::relocalizing{false};
std::atomic<bool> app::cox_first_fix{false};
std::atomic<bool> app::motors_ready{false};
std::vector<std::thread> threads;

double ROBOT_SPEED = 1.875;

static void handle_sigint(int) {
    if (app::shutdown.exchange(true)) {
        struct timespec ts = { 0, 100 * 1000 * 1000 };
        nanosleep(&ts, nullptr);
        _exit(130);
    }
    LOGE("[SYSTEM]") << "[SIGINT] shutting down…\n";
}

void wait_for_threads() {
    for (auto &t : threads) {
        if (t.joinable()) t.join();
    }
}

#if CALIBRATION_BUILD

int main() {
    std::signal(SIGINT, handle_sigint);

    constexpr double TARGET_MM   = 300.0;
    constexpr double DRIVE_MM_S  = 120.0;   // cruise speed during roll

    // ROBOT_SPEED still drives the motor's slew limiter, so pull it from
    // config the same way the mission main does. A typo here just biases
    // the brake-distance estimate.
    Config cfg;
    if (load_config("config.yml", cfg)) {
        double rs = cfg.get_dbl("robot.speed", ROBOT_SPEED);
        if (std::isfinite(rs) && rs > 0.0) {
            ROBOT_SPEED = std::min(rs, 5.0);
        }
    }
    LOG("Cal") << "ROBOT_SPEED=" << ROBOT_SPEED
               << "  target=" << TARGET_MM << " mm"
               << "  drive=" << DRIVE_MM_S << " mm/s";

    Motor motor;
    if (!motor.start()) {
        LOGE("Cal") << "motor.start() failed";
        return 1;
    }

    double l0 = 0.0, r0 = 0.0;
    motor.get_encoders(l0, r0);
    LOG("Cal") << "Seed encoders L=" << l0 << " mm  R=" << r0 << " mm";

    // ACCEL_V_BASE_MM_S2 mirrors the constant in src/motor.cpp; the slew
    // limiter ramps down at this rate × ROBOT_SPEED. Pre-command the stop
    // one brake-window early so the chassis coasts to ~TARGET_MM rather
    // than overshooting by the full ramp-down distance.
    constexpr double ACCEL_V_BASE_MM_S2 = 150.0;
    const double decel      = ACCEL_V_BASE_MM_S2 * ROBOT_SPEED;
    const double brake_dist = (DRIVE_MM_S * DRIVE_MM_S) / (2.0 * decel);
    const double stop_when  = std::max(0.0, TARGET_MM - brake_dist);
    LOG("Cal") << "brake_dist=" << brake_dist
               << " mm  stop_cmd_at=" << stop_when << " mm";

    motor.set_velocity(DRIVE_MM_S, 0.0);

    bool stopped = false;
    while (!app::shutdown.load()) {
        double l = 0.0, r = 0.0;
        motor.get_encoders(l, r);
        const double dL = l - l0;
        const double dR = r - r0;
        const double avg = 0.5 * (dL + dR);

        if (!stopped && avg >= stop_when) {
            motor.set_velocity(0.0, 0.0);
            stopped = true;
            LOG("Cal") << "stop cmd @ avg=" << avg
                       << " mm (L=" << dL << "  R=" << dR << ")";
        }
        if (stopped) {
            double v = 0.0, w = 0.0;
            motor.get_current_v_w(v, w);
            if (std::fabs(v) < 0.5) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Let any residual slew + wheel free-roll bleed off before sampling.
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    double l1 = 0.0, r1 = 0.0;
    motor.get_encoders(l1, r1);
    const double dL = l1 - l0;
    const double dR = r1 - r0;

    std::cout << "\n=== 300 mm calibration roll ===\n"
              << "  L reported : " << dL << " mm\n"
              << "  R reported : " << dR << " mm\n"
              << "  Mean       : " << 0.5 * (dL + dR) << " mm\n\n"
              << "Tape-measure the chassis travel A_mm, then update motor.hpp:\n"
              << "  LEFT_TICKS_PER_MM  = "
              << Motor::LEFT_TICKS_PER_MM  << " * " << dL << " / A_mm\n"
              << "  RIGHT_TICKS_PER_MM = "
              << Motor::RIGHT_TICKS_PER_MM << " * " << dR << " / A_mm\n"
              << std::flush;

    motor.stop();
    return 0;
}

#else  // ---- mission main, preserved for `git checkout main.cpp`-free revert ----

#include <dashboard.hpp>
#include <core.hpp>
#include <data_logger.hpp>
#include <tracker.hpp>
#include <kalman.hpp>
#include <telemetry.hpp>
#include <utils.hpp>

int main() {
        std::signal(SIGINT, handle_sigint);

    // loading configs
    std::string config_path = "config.yml";

    Config cfg;
    if (!load_config(config_path, cfg)) {
        LOG("Main") << "No " << config_path << " found, using built-in defaults.";
    } else {
        LOG("Main") << "Loaded config from " << config_path
                    << " (" << cfg.raw().size() << " keys).";
    }

    {
        double rs = cfg.get_dbl("robot.speed", ROBOT_SPEED);
        if (!std::isfinite(rs) || rs <= 0.0) {
            LOGE("Main") << "robot.speed = " << rs
                         << " is invalid; falling back to default "
                         << ROBOT_SPEED;
        } else {
            if (rs > 5.0) {
                LOGE("Main") << "robot.speed = " << rs
                             << " is above the soft ceiling (5.0); clamping. "
                                "Accel ramps & omega will saturate at chassis caps.";
                rs = 5.0;
            }
            ROBOT_SPEED = rs;
        }
        LOG("Main") << "ROBOT_SPEED = " << ROBOT_SPEED;
    }
    CameraConfig cam_cfg(
        cfg.get_int("camera.width",      640),
        cfg.get_int("camera.height",     480),
        cfg.get_int("camera.fps_target",  15));
    cam_cfg.focal_length_px = cfg.get_dbl(
        "camera.focal_length_px", 650.0);
    cam_cfg.grab_bottom_region_ratio = cfg.get_dbl(
        "camera.grab_bottom_region_ratio", cam_cfg.grab_bottom_region_ratio);
    cam_cfg.grab_bottom_fill_ratio = cfg.get_dbl(
        "camera.grab_bottom_fill_ratio", cam_cfg.grab_bottom_fill_ratio);
    cam_cfg.grab_angle_tol_deg = cfg.get_dbl(
        "camera.grab_angle_tol_deg", cam_cfg.grab_angle_tol_deg);
    cam_cfg.capture_distance_mm = cfg.get_dbl(
        "camera.capture_distance_mm", cam_cfg.capture_distance_mm);
    cam_cfg.capture_bearing_tol_rad = cfg.get_dbl(
        "camera.capture_bearing_tol_rad", cam_cfg.capture_bearing_tol_rad);
    cam_cfg.retention_distance_mm = cfg.get_dbl(
        "camera.retention_distance_mm", cam_cfg.retention_distance_mm);
    cam_cfg.show_debug_view = cfg.get_bool(
        "camera.show_debug_view", cam_cfg.show_debug_view);

    MissionConfig mission;
    mission.home_margin   = cfg.get_dbl("mission.home_margin_mm",  mission.home_margin);
    mission.arm_length_mm = cfg.get_dbl("mission.arm_length_mm",   mission.arm_length_mm);

    {
        std::string g = cfg.get_str("mission.goto", "");
        if (!g.empty()) {
            double gx = 0.0, gy = 0.0, gth_deg = 0.0;
            if (std::sscanf(g.c_str(), "%lf , %lf , %lf",
                            &gx, &gy, &gth_deg) == 3) {
                mission.goto_enabled   = true;
                mission.goto_x_mm      = gx;
                mission.goto_y_mm      = gy;
                mission.goto_theta_rad = gth_deg * M_PI / 180.0;
                LOG("Main") << "GOTO target: (" << gx << ", " << gy
                            << ", " << gth_deg << " deg)";
            } else {
                LOGE("Main") << "mission.goto = \"" << g
                             << "\" — expected \"x,y,theta_deg\". "
                                "Falling back to default mission.";
            }
        }
    }

    RansacParams map_ransac;
    map_ransac.inlier_thresh_mm    = cfg.get_dbl("map.ransac.inlier_thresh_mm",    25.0);
    map_ransac.max_iters_per_line  = cfg.get_int("map.ransac.max_iters_per_line",  400);
    map_ransac.inlier_points_req   = cfg.get_int("map.ransac.inlier_points_req",   30);
    map_ransac.min_inlier_fraction = cfg.get_dbl("map.ransac.min_inlier_fraction", 0.06);
    map_ransac.max_lines           = cfg.get_int("map.ransac.max_lines",           6);
    map_ransac.endpoint_padding_mm = cfg.get_dbl("map.ransac.endpoint_padding_mm", 50.0);
    map_ransac.min_segment_len_mm  = cfg.get_dbl("map.ransac.min_segment_len_mm",  600.0);

    LOG("Main") << "Map RANSAC: max_lines=" << map_ransac.max_lines
                << " min_seg=" << map_ransac.min_segment_len_mm
                << " mm thresh=" << map_ransac.inlier_thresh_mm << " mm";

    telemetry::init(
        cfg.get_str ("logger.udp_host",      "127.0.0.1"),
        cfg.get_int ("logger.udp_port",      5005),
        cfg.get_bool("logger.udp_enabled",   false),
        cfg.get_int ("logger.scan_subsample", 2));

    DataLogger data_logger;
    data_logger.open(
        cfg.get_bool("logger.files_enabled", false),
        cfg.get_str ("logger.out_dir",       "logs"));

    FilterResult filter;
    if (load_filter_result(cfg, filter)) {
        LOG("Main") << "Loaded filter calibration from " << config_path;
    } else {
        LOG("Main") << "No filter calibration in config — running calibration.";
        CalibrateCamera calibrator(cam_cfg);
        filter = calibrator.start_filter_calibration();
        if (filter.ready) {
            if (save_filter_result(config_path, filter)) {
                LOG("Main") << "Saved filter calibration to " << config_path;
            } else {
                LOG("Main") << "Failed to write filter calibration to " << config_path;
            }
            std::cout << "Waiting 1 seconds before starting . ..\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } else {
            LOG("Main") << "Calibration aborted; filter not saved.";
        }
    }
    if (!filter.ready) {
        LOGE("Main") << "Color filter is not ready -- vision cannot run. "
                        "Refusing to start.";
        return 1;
    }

    INA219 power_interface = configure_voltage_interface();
    Dashboard dash;
    TrackerResult target;
    LidarData lidar_data;
    KalmanFilter kf;
    dash.start(48);

    if (cfg.get_bool("battery.enabled", true)) {
        dash.add_row("Battery");
        threads.emplace_back(loop_check_battery,
            std::ref(power_interface), std::ref(dash));
    } else {
        LOG("Main") << "Battery monitor disabled via config.";
    }

    dash.add_row("Cox");
    threads.emplace_back(loop_localization,
        std::ref(dash), std::ref(lidar_data), std::ref(kf),
        std::cref(map_ransac), std::ref(data_logger));
    std::this_thread::sleep_for(std::chrono::seconds(2));

    dash.add_row("Vision");
    threads.emplace_back(loop_vision_tracking,
        std::ref(dash), filter, cam_cfg, std::ref(target));
    dash.render();
    dash.add_row("Motor");
    dash.add_row("Odometry");
    dash.add_row("KF");
    dash.add_row("Mission");
    threads.emplace_back(loop_control,
        std::ref(dash), std::ref(kf), std::ref(target),
        std::ref(lidar_data), std::cref(mission), std::ref(data_logger));

    while(!app::shutdown) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        dash.render();
    }
    wait_for_threads();
    data_logger.close();
    telemetry::shutdown();
}

#endif  // CALIBRATION_BUILD
