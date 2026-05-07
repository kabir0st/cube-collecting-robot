#pragma once
#include <deque>
#include <mutex>
#include <vector>

#include <Eigen/Core>

#include <types.hpp>
#include <loging.hpp>

// Output of one Cox-matcher solve.
struct PoseEstimate {
    Pose2D pose;          // (x mm, y mm, theta rad)
    float  cov[9];        // 3x3 row-major covariance over (x, y, theta)
    float  sigma_x;       // sqrt(cov[0])
    float  sigma_y;       // sqrt(cov[4])
    float  sigma_theta;   // sqrt(cov[8])  (rad)
    int    iterations;    // Newton iterations performed
    int    inliers;       // scan points that passed the residual gate
    bool   converged;     // step size dropped below tolerance
};


// Cox '91 line-based scan matcher with a built-in rolling frame buffer.
//
// Threading: process_point() is the producer (lidar listener thread);
// estimate() is the consumer (main thread, ~1 Hz). The mutex protects
// `frames_` and `in_progress_`.
class PositionEstimator {
public:
    PositionEstimator(const std::vector<LineSeg>& ref_map,
                      Pose2D initial_pose,
                      int    frames_window,
                      float  sensor_noise_mm,
                      Loging* logger = nullptr);

    void process_point(int quality, int angle, int distance);
    PoseEstimate estimate();

    Pose2D current() const { return current_pose_; }

    void set_max_iters(int n) { max_iters_ = n; }
    void set_gate_k(float k)  { gate_k_ = k; }
    void set_sigma_r(float s) { sigma_r_ = s; }

private:
    std::vector<LineSeg> walls_;
    Pose2D  current_pose_;
    int     frames_window_;
    int     max_iters_;
    float   sigma_r_;
    float   gate_k_;
    float   tol_dxy_;
    float   tol_dtheta_;

    std::mutex                 buf_mx_;
    std::deque<LidarDataFrame> frames_;
    LidarDataFrame             in_progress_;
    int                        last_angle_;

    Loging* logger_;

    // Pre-allocated to avoid heap traffic in the 1 Hz hot path.
    Eigen::Matrix2Xf P_lidar_;   // 2 x N : scan in lidar frame
    Eigen::Matrix2Xf P_world_;   // 2 x N : scan transformed per iteration
};


// Top-level driver: starts the listener + lidar, polls estimate() at
// 1 Hz until app::shutdown is set, prints pose + uncertainty.
void do_estimate_position(const std::vector<LineSeg>& ref_map,
                          float sensor_noise_mm,
                          Loging* logger = nullptr);
