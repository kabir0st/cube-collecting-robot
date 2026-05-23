#pragma once

#include <chrono>
#include <fstream>
#include <mutex>
#include <string>

#include <types.hpp>

// Lab-3 file logger: three CSV streams (odometry, scan-match, kalman)
// sharing a single monotonic time base. Methods short-circuit when the
// logger was opened with enabled=false, so callers can sprinkle log_*
// calls unconditionally.
class DataLogger {
public:
    DataLogger() = default;
    ~DataLogger();

    // Returns true on success (including the disabled no-op path). When
    // enabled, creates out_dir if needed and opens three timestamped CSVs.
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
