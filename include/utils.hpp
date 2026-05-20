#pragma once

#include <cmath>
#include <map>
#include <vector>

#include <Lidar.hpp>
#include <types.hpp>

// for voltage tracking
#include "ina219.h"
#include "spi_com.h"

bool start_lidar(hh::Lidar& lidar);
void stop_lidar(hh::Lidar& lidar);
void loop_collect_frames(hh::Lidar& lidar, LidarData& frames);

std::map<double, std::vector<double>> get_angle_mapped_lidar_data(LidarData& lidar_data);

std::pair<std::vector<double>, std::vector<double>> get_latest_frame_lidar_data(LidarData& lidar_data);

INA219 configure_voltage_interface();

std::string get_timestamp();

inline double wrap_pi(double a) {
    while (a >   M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}