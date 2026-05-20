#include <ina219.h>
#include <dashboard.hpp>
#include <types.hpp>
#include <kalman.hpp>

void loop_check_battery(INA219 &power_interface, Dashboard& dash);
void loop_localization(Dashboard& dash,
                       LidarData& lidar_data, KalmanFilter& kf);
void loop_control(Dashboard &dash,
                  LidarData &lidar_data, KalmanFilter& kf);

// void loop_control(Dashboard& dash, KalmanFilter& kf,
//                   PurePursuitController& controller,
//                   StateMachine& sm,
//                   LidarData& lidar_data,
//                   PotentialFieldAvoider& apf,
//                   DataLogger& logger);
