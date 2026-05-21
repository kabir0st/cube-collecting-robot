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

    // Wheel-noise model. The old code used a random-walk form (var = k * |d|),
    // which gives sigma ~ sqrt(d). Real wheel slip + radius miscalibration is
    // a *scale error* (sigma ~ d), so squared variance must grow as d^2.
    // Calibrated against the observed "report 600 mm, actually 610-620 mm"
    // behaviour: that's ~2-3% scale error on the chassis distance.
    //
    // Three independent error sources, all expressed as 1-sigma fractions:
    //
    //   K_WHEEL_INDEP : per-wheel independent slip. Drives heading drift
    //                   (the only way left/right wheels can disagree).
    //   K_COMMON      : both-wheels-the-same slip (battery sag, surface
    //                   compression, wheel-radius error). Affects d but
    //                   cancels in theta because both wheels drift together.
    //   K_ROT         : extra scale error specific to commanded rotation
    //                   (lateral wheel scrubbing during in-place turns).
    //
    // Floor terms keep variance > 0 at very small steps (encoder quantization
    // is real but small: TICKS_PER_MM = 103.8 -> ~10 micron per tick).
    constexpr double K_WHEEL_INDEP = 0.010;   // 1% per-wheel independent slip
    constexpr double K_COMMON      = 0.025;   // 2.5% common-mode scale error
    constexpr double K_ROT         = 0.030;   // 3% scale on |d_theta|
    constexpr double VAR_L_FLOOR_MM2 = 0.25;  // mm^2 per step
    constexpr double VAR_THETA_FLOOR = 1.0e-6;// rad^2 per step

    const double var_L = K_WHEEL_INDEP * K_WHEEL_INDEP * dL_mm * dL_mm
                       + VAR_L_FLOOR_MM2;
    const double var_R = K_WHEEL_INDEP * K_WHEEL_INDEP * dR_mm * dR_mm
                       + VAR_L_FLOOR_MM2;

    // (dL, dR) -> (d, d_theta) is J = [[1/2, 1/2], [-1/b, 1/b]].
    // var_d        = 1/4 * (var_L + var_R)        (was 0.08 -- buggy)
    // var_theta    = (var_L + var_R) / b^2
    // cov(d,theta) = (var_R - var_L) / (2 b)
    const double var_d_wheel     = 0.25 * (var_L + var_R);
    const double var_theta_wheel = (var_L + var_R) /
                                   (Motor::WHEELBASE_MM * Motor::WHEELBASE_MM);

    // Common-mode scale error is fully correlated across wheels: cancels in
    // theta but stacks in d.
    const double var_d_common = (K_COMMON * step.d_mm) * (K_COMMON * step.d_mm);
    // Rotation scale error couples to |d_theta| only.
    const double var_theta_rot = (K_ROT * step.d_theta) * (K_ROT * step.d_theta);

    step.var_d       = var_d_wheel + var_d_common;
    step.var_theta   = var_theta_wheel + var_theta_rot + VAR_THETA_FLOOR;
    step.cov_d_theta = (var_R - var_L) / (2.0 * Motor::WHEELBASE_MM);
    step.valid = true;
    return step;
}
