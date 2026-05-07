#pragma once
#include <map>
#include <mutex>
#include <vector>

#include <types.hpp>
#include <mapping.hpp>
#include <loging.hpp>

// Stationary-lidar calibration:
//   1. process_point() collects raw scans into per-angle bins.
//   2. calibrate() pools per-bin variance into one sigma estimate (mm).
//   3. create_reference_map() averages each bin and runs RANSAC line
//      extraction on the resulting cartesian point cloud.
//
// Thread-safety:
//   process_point() is the producer (lidar listener thread).
//   calibrate() and create_reference_map() are consumers (main thread,
//   after stop_stream()). The internal mutex protects against the
//   subtle case where the listener is still inside process_point at
//   the moment stop_stream() returns.
class LidarCalibration {
public:
    explicit LidarCalibration(Loging* logger = nullptr)
        : lidar_sigma_(0.0f), logger_(logger) {}

    // Producer-side hook for ListenerThread.
    void process_point(int quality, int angle, int distance);

    // Pooled standard deviation of distance measurements within each
    // angle bin. Returns the value and stores it for create_reference_map.
    float calibrate();

    // Run RANSAC on the averaged scan to extract wall segments.
    // Throws std::runtime_error if zero walls were found.
    std::vector<LineSeg> create_reference_map();

    float sensor_noise() const { return lidar_sigma_; }

private:
    std::mutex                       mx_;
    std::map<int, std::vector<int>>  scan_collection_;
    float                            lidar_sigma_;
    Loging*                          logger_;
};
