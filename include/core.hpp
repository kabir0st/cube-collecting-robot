#pragma once

#include <ina219.h>
#include <dashboard.hpp>
#include <data_logger.hpp>
#include <types.hpp>
#include <kalman.hpp>
#include <calibrations.hpp>

struct MissionConfig {
    double home_margin   = 100;
    double arm_length_mm = 80.0;
    bool   goto_enabled    = false;
    double goto_x_mm       = 0.0;
    double goto_y_mm       = 0.0;
    double goto_theta_rad  = 0.0;
};

void loop_check_battery(INA219 &power_interface, Dashboard& dash);
void loop_localization(Dashboard& dash,
                       LidarData& lidar_data, KalmanFilter& kf,
                       const RansacParams& ransac,
                       DataLogger& logger);
void loop_control(Dashboard &dash,
                KalmanFilter& kf,
                TrackerResult& target,
                LidarData& lidar_data,
                const MissionConfig& mission,
                DataLogger& logger);
