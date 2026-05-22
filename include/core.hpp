#pragma once

#include <ina219.h>
#include <dashboard.hpp>
#include <types.hpp>
#include <kalman.hpp>
#include <calibrations.hpp>

struct MissionConfig {
    // Home is hardcoded as the robot's starting pose (0, 0, 0). Only the
    // zone radius and arm geometry are tunable.
    double home_margin   = 152.4;
    double arm_length_mm = 80.0;
};

void loop_check_battery(INA219 &power_interface, Dashboard& dash);
void loop_localization(Dashboard& dash,
                       LidarData& lidar_data, KalmanFilter& kf,
                       const RansacParams& ransac);
void loop_control(Dashboard &dash,
                KalmanFilter& kf,
                TrackerResult& target,
                LidarData& lidar_data,
                const MissionConfig& mission);
