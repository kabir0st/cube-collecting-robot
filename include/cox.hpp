#pragma once

#include <vector>
#include <Eigen/Core>
#include <mutex>
#include <types.hpp>
#include <dashboard.hpp>
#include <data_logger.hpp>
#include <kalman.hpp>

class CoxScanMatchAlgorithm {
    private:
        std::vector<LineSeg> walls;
        int max_itr;
        LidarData &lidar_data;

        float sigma_r;
        float gate_k;
        float tol_d_xy;
        float tol_d_a;

        Eigen::Matrix2Xf pts_lidar;
        Eigen::Matrix2Xf pts_world;

    public:
        CoxScanMatchAlgorithm(
            const std::vector<LineSeg> &map,
            LidarData& lidar_data);
        PosEstimate estimate(Pos2D init_pos);
};

void loop_scan_cox_estimation(
    std::vector<LineSeg>& ref_map, LidarData& lidar_data,
    Dashboard& dash, KalmanFilter& kf, DataLogger& logger);