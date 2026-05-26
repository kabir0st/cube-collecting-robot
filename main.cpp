#include <cmath>
#include <cstdio>
#include <csignal>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <global.hpp>
#include <types.hpp>
#include <loggers.hpp>
#include <utils.hpp>
#include <dashboard.hpp>
#include <calibrations.hpp>
#include <core.hpp>
#include <data_logger.hpp>
#include <tracker.hpp>
#include <kalman.hpp>
#include <telemetry.hpp>

std::atomic<bool> app::shutdown{false};
std::atomic<bool> app::lidar_ready{false};
std::atomic<bool> app::lidar_stop{false};
std::atomic<bool> app::obstacle_ahead{false};
std::atomic<double> app::obstacle_left_mm{std::numeric_limits<double>::infinity()};
std::atomic<double> app::obstacle_right_mm{std::numeric_limits<double>::infinity()};
std::atomic<bool> app::relocalizing{false};
std::atomic<bool> app::cox_first_fix{false};
std::atomic<bool> app::motors_ready{false};
std::atomic<double> app::v_cmd_last_mm_s{0.0};
std::atomic<double> app::w_cmd_last_rad_s{0.0};
std::vector<std::thread> threads;

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

    FilterResult target_filter;
    const bool filter_ready  = cfg.get_bool("filter.ready", false);
    const bool target_loaded = filter_ready && load_filter_result(cfg, target_filter);

    if (!filter_ready || !target_loaded) {
        if (filter_ready && !target_loaded) {
            LOGE("Main") << "filter.ready=true but filter.target is missing/"
                            "malformed. Set filter.ready=false in config.yml "
                            "and re-run to recalibrate.";
            return 1;
        }
        LOG("Main") << "No filter calibration — running calibration.";
        CalibrateCamera calibrator(cam_cfg);
        target_filter = calibrator.start_filter_calibration();
        if (!target_filter.ready) {
            LOGE("Main") << "Target calibration aborted; nothing saved.";
            return 1;
        }
        if (!save_filter_section(config_path, target_filter, /*ready=*/true)) {
            LOGE("Main") << "Failed to write filter calibration to "
                         << config_path;
            return 1;
        }
        LOG("Main") << "Saved target calibration. "
                       "Restart the app to run the mission.";
        return 0;
    }

    LOG("Main") << "Loaded filter calibration from " << config_path;

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
        std::cref(map_ransac), std::ref(data_logger));

    dash.add_row("Vision");
    threads.emplace_back(loop_vision_tracking,
        std::ref(dash), target_filter,
        cam_cfg, std::ref(target));
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
    // shutdown
    wait_for_threads();
    data_logger.close();
    telemetry::shutdown();
}
