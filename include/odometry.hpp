#pragma once
#include <cstdint>
#include <mutex>

// Single-step motion increment + its diagonal noise
// `valid = false` is returned during the seeding phase (first call after reset)
// when no delta is available yet.
struct OdomStep {
    double d_mm = 0.0; // forward distance (signed)
    double d_theta = 0.0; // heading change in rad (signed)
    double var_d = 0.0; // mm^2
    double var_theta = 0.0; // rad^2
    // (d, d_theta) cross-covariance.
    double cov_d_theta = 0.0;  // mm*rad
    bool   valid = false;
};

// Differential-drive odometry step generator. Inputs are cumulative per-wheel
// distances in mm (forward = +); outputs are per-cycle (d, d_theta) plus their noise
class Odometry {
public:
    Odometry();
    void reset(double left_mm, double right_mm);
    OdomStep update(double left_mm, double right_mm);
private:
    void _seed_locked(double l_mm, double r_mm);
    static constexpr int MED_N = 3;
    mutable std::mutex _m;
    bool    _seeded = false;
    double  _l_buf[MED_N] = {0.0, 0.0, 0.0};
    double  _r_buf[MED_N] = {0.0, 0.0, 0.0};
    int     _buf_idx = 0;
    double  _last_l_med = 0.0;
    double  _last_r_med = 0.0;
};
