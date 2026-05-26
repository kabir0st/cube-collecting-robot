#pragma once

#include <chrono>
#include <fstream>
#include <mutex>
#include <string>

#include <types.hpp>

class DataLogger {
public:
    DataLogger() = default;
    ~DataLogger();

    bool open(bool enabled, const std::string& out_dir);
    void close();

    void log_odom  (double t_s, const Pos2D& p, const Pose2DCov& cov);
    void log_kalman(double t_s, const Pos2D& p, const Pose2DCov& cov);
    void log_cox   (double t_s, const PosEstimate& est);

    double now_s() const;
    bool enabled() const { return _enabled; }

private:
    std::ofstream _odom;
    std::ofstream _cox;
    std::ofstream _kalman;
    std::mutex    _m;
    std::chrono::steady_clock::time_point _t0;
    bool _enabled = false;
};
