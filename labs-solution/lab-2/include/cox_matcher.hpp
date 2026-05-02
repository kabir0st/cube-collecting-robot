#pragma once
#include <vector>
#include <types.hpp>

// Cox '91 line-based scan matcher. One match() call iterates the
// linearization from a prior pose against a fixed line map and returns
// the updated pose plus full 3x3 covariance. Single-threaded; intended
// to be called once per revolution at ~1 Hz on RPi 3B.
class CoxScanMatcher {
public:
    CoxScanMatcher(std::vector<LineSeg> map, float sigma_r);

    TrackerResult match(const LidarDataFrame& scan, const Pose2D& prior);

    void set_max_iters(int n) { max_iters_ = n; }
    void set_gate_k(float k)  { gate_k_ = k; }

private:
    std::vector<LineSeg> map_;
    float sigma_r_;
    int   max_iters_;
    float gate_k_;
    float tol_dtheta_;
    float tol_dxy_;

    int  nearest_segment(float pwx, float pwy, float& y_signed) const;
    bool one_iteration(const LidarDataFrame& scan, Pose2D& pose,
                       float A[9], float& s2, int& n_in) const;
};
