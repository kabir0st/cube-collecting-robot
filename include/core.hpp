#pragma once

#include <ina219.h>
#include <dashboard.hpp>
#include <types.hpp>
#include <kalman.hpp>
#include <calibrations.hpp>

struct MissionConfig {
    // Home is centered on the robot's starting pose (0, 0). home_margin is
    // the arrival radius (0.5 ft ≈ 152.4 mm), making the full home zone
    // 1 ft x 1 ft around the starting point.
    double home_x_mm     = 0.0;
    double home_y_mm     = 0.0;
    double home_theta    = 0.0;
    double home_margin   = 152.4;
    double arm_length_mm = 80.0;
};

void loop_check_battery(INA219 &power_interface, Dashboard& dash);
void loop_localization(Dashboard& dash,
                       LidarData& lidar_data, KalmanFilter& kf,
                       const RansacParams& ransac, double home_theta);
void loop_control(Dashboard &dash,
                KalmanFilter& kf,
                TrackerResult& target,
                LidarData& lidar_data,
                const MissionConfig& mission);
