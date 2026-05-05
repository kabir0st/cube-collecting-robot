#pragma once
#include <iostream>
#include <types.hpp>
#include <mapping.hpp>
#include <map>
#include <vector>




class LidarCalibration {
    private:
        std::map<int, std::vector<int>> scan_collection;
        float lidar_sigma;

    public:
        LidarCalibration() : lidar_sigma(0.0f) {}
        float calibrate();
        void process_point(int quality, int angle, int distance);
        Rectangle create_reference_map();
        float sensor_noise() const { return lidar_sigma; }
        const std::map<int, std::vector<int>> &samples() const { return scan_collection; }
};