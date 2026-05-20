#pragma once
#include <mutex>
#include <types.hpp>
#include <odometry.hpp>

// Extended Kalman Filter on (x_mm, y_mm, theta_rad).
// Predict:  midpoint-heading diff-drive motion model (theta_m = theta + dtheta/2).
// Correct:  H = I (Cox measures the pose directly).
class KalmanFilter {
public:
    KalmanFilter();

    // Reset the state.
    void reset(const Pos2D& pose = {}, const Pose2DCov& cov = default_cov());

    // EKF prediction with midpoint heading.
    void predict(double d_mm, double d_theta,
                 double var_d, double var_theta,
                 double cov_d_theta = 0.0);

    // EKF correction;
    void correct(const Pos2D& z, const Pose2DCov& R);

    Pos2D pose() const;
    Pose2DCov covariance() const;
    // Snapshot of both (pose, covariance) under a single lock acquisition.
    // using this for logging
    void snapshot(Pos2D& out_pose, Pose2DCov& out_cov) const;

    static Pose2DCov default_cov();

private:
    mutable std::mutex _m;
    Pos2D _pose;
    Pose2DCov _cov;
};
