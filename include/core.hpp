#include <ina219.h>
#include <dashboard.hpp>
#include <types.hpp>
#include <kalman.hpp>
#include <calibrations.hpp>

void loop_check_battery(INA219 &power_interface, Dashboard& dash);
void loop_localization(Dashboard& dash,
                       LidarData& lidar_data, KalmanFilter& kf,
                       const ArenaConfig& arena, double home_theta);
void loop_control(Dashboard &dash,
                KalmanFilter& kf,
                TrackerResult& target);
