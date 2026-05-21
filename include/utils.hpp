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

inline float lipo_soc_pct(float pack_v, int cells = 3) {
    const float vc = pack_v / cells;
    static constexpr float t[][2] = {
        {4.20f,100.f},{4.10f, 90.f},{4.00f, 80.f},{3.93f, 70.f},
        {3.87f, 60.f},{3.82f, 50.f},{3.79f, 40.f},{3.77f, 30.f},
        {3.74f, 20.f},{3.68f, 10.f},{3.45f,  5.f},{3.00f,  0.f},
    };
    if (vc >= t[0][0]) return 100.f;
    constexpr int n = sizeof(t)/sizeof(t[0]);
    if (vc <= t[n-1][0]) return 0.f;
    for (int i = 1; i < n; ++i) {
        if (vc >= t[i][0]) {
            const float v1 = t[i-1][0], p1 = t[i-1][1];
            const float v0 = t[i][0],   p0 = t[i][1];
            return p0 + (p1 - p0) * (vc - v0) / (v1 - v0);
        }
    }
    return 0.f;
}