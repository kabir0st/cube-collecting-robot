#pragma once
#include <iostream>
#include <types.hpp>
#include <map>
#include <vector>




class LidarCalibration {
    private:
        std::map<int, std::vector<int>> scan_collection;
        float lidar_sigma;
    
    public:
        float calibrate();
        void collect_points(int quality, int angle, int distance);
        const std::map<int, std::vector<int>>& samples() const { return scan_collection; }
};