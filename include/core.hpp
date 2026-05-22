#pragma once

#include <ina219.h>
#include <dashboard.hpp>
#include <types.hpp>
#include <kalman.hpp>
#include <calibrations.hpp>

struct MissionConfig {
    double home_x_mm     = 275.0;
    double home_y_mm     = 0.0;
    double home_theta    = 0.0;
    double home_margin   = 25.0;
    double arm_length_mm = 80.0;
};

void loop_check_battery(INA219 &power_interface, Dashboard& dash);
void loop_localization(Dashboard& dash,
                       LidarData& lidar_data, KalmanFilter& kf,
                       const ArenaConfig& arena, double home_theta);
void loop_control(Dashboard &dash,
                KalmanFilter& kf,
                TrackerResult& target,
                LidarData& lidar_data,
                const MissionConfig& mission);
