#include <thread>
#include <mutex>
#include <csignal>

#include <global.hpp>
#include <types.hpp>
#include <loggers.hpp>
#include <utils.hpp>
#include <dashboard.hpp>
#include <calibrations.hpp>
#include <core.hpp>
#include <tracker.hpp>
#include <kalman.hpp>
#include <telemetry.hpp>

std::atomic<bool> app::shutdown{false};
std::atomic<bool> app::lidar_ready{false};
std::atomic<bool> app::lidar_stop{false};
std::atomic<bool> app::obstacle_ahead{false};
std::atomic<bool> app::relocalizing{false};
std::atomic<bool> app::cox_first_fix{false};
std::vector<std::thread> threads;

// Definition for the extern declared in global.hpp. Boot default is the
// previous compile-time tuning (1.875); main() overwrites this from
// `robot.speed` in config.yml before any motor / controller object is
// constructed, so member initializers that read ROBOT_SPEED see the
// configured value.
double ROBOT_SPEED = 1.875;


static void handle_sigint(int) {
    if (app::shutdown.exchange(true)) {
        // giving a chance for the 20ms motor loop to sleep
        struct timespec ts = { 0, 100 * 1000 * 1000 };
        nanosleep(&ts, nullptr);
        _exit(130);
    }
    LOGE("[SYSTEM]") << "[SIGINT] shutting down…\n";
}

void wait_for_threads() {
    for (auto &t : threads)
    {
        if (t.joinable()) t.join();
    }
}

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

    // Apply ROBOT_SPEED FIRST, before any object that captures it at
    // construction time (Motor, PurePursuit, StateMachine — all built down
    // inside loop_control). Clamp to a sane range so a typo in config
    // doesn't ramp accel into orbit; logs the resolved value so the
    // operator can see what's actually in effect.
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
    cam_cfg.grab_bottom_width_ratio = cfg.get_dbl(
        "camera.grab_bottom_width_ratio", cam_cfg.grab_bottom_width_ratio);
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

    // RANSAC parameters for the boot-time lidar map build. Defaults tuned
    // for a rectangular arena with 1-2 other robots inside (see config.yml
    // map.ransac block for the rationale per key).
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

    // start rest of the system
    INA219 power_interface = configure_voltage_interface();
    Dashboard dash;
    // shared datas
    TrackerResult target;
    LidarData lidar_data;
    KalmanFilter kf;
    // 9 panels × 4 lines each = 36 lines (+ slack).
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
        std::cref(map_ransac));
    // waiting for cox 1st seeds to be stable
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
        std::ref(lidar_data), std::cref(mission));

    while(!app::shutdown) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        dash.render();
    }
    // shutdown
    wait_for_threads();
    telemetry::shutdown();
}