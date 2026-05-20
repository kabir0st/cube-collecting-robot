#include <odometry.hpp>

#include <algorithm>
#include <cmath>
#include <motor.hpp>

namespace {
    inline double median3(double a, double b, double c) {
        if (a > b) std::swap(a, b);
        if (b > c) std::swap(b, c);
        if (a > b) std::swap(a, b);
        return b;
    }
}

Odometry::Odometry() = default;

void Odometry::_seed_locked(double l_mm, double r_mm) {
    for (int i = 0; i < MED_N; ++i) {
        _l_buf[i] = l_mm;
        _r_buf[i] = r_mm;
    }
    _buf_idx     = 0;
    _last_l_med  = l_mm;
    _last_r_med  = r_mm;
    _seeded      = true;
}

void Odometry::reset(double left_mm, double right_mm) {
    std::lock_guard<std::mutex> lk(_m);
    _seed_locked(left_mm, right_mm);
}

OdomStep Odometry::update(double left_mm, double right_mm) {
    std::lock_guard<std::mutex> lk(_m);
    OdomStep step;
    if (!_seeded) {
        _seed_locked(left_mm, right_mm);
        return step;  // valid = false
    }

    _l_buf[_buf_idx] = left_mm;
    _r_buf[_buf_idx] = right_mm;
    _buf_idx = (_buf_idx + 1) % MED_N;

    const double l_med = median3(_l_buf[0], _l_buf[1], _l_buf[2]);
    const double r_med = median3(_r_buf[0], _r_buf[1], _r_buf[2]);

    const double dL_mm = l_med - _last_l_med;
    const double dR_mm = r_med - _last_r_med;
    _last_l_med = l_med;
    _last_r_med = r_med;

    step.d_mm   = 0.5 * (dL_mm + dR_mm);
    step.d_theta = (dR_mm - dL_mm) / Motor::WHEELBASE_MM;

    // Per-wheel variance grows with distance traveled; floors add fixed
    // per-step noise to capture systematic effects (slip, geometry error).
    const double var_L = _k_left  * std::fabs(dL_mm);
    const double var_R = _k_right * std::fabs(dR_mm);
    // just guessing noise haven't measured it
    step.var_d     = 0.08 * (var_L + var_R);
    step.var_theta = (var_L + var_R) / (Motor::WHEELBASE_MM * Motor::WHEELBASE_MM);
    // Cross-covariance between d and d_theta induced by the (dL, dR) -> (d, d_theta)
    // transform J = [[1/2, 1/2], [-1/b, 1/b]]:
    //   cov(d, d_theta) = (var_R - var_L) / (2 * b)
    // Sign tracks turn direction; magnitude is 0 for symmetric wheel motion.
    step.cov_d_theta = (var_R - var_L) / (2.0 * Motor::WHEELBASE_MM);
    step.valid = true;
    return step;
}
