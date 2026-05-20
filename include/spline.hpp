#pragma once
#include <vector>

#include <types.hpp>

// Generate a B-cubic spline path from a sequence of control points.
//
// The first and last control points are duplicated at the ends (phantom
// padding) so the resulting curve actually passes through them. Interior
// control points are smoothed-through, not interpolated.
//
// `default_v_max` is interpreted as the *cruise* speed: each output point's
// `v_max` is shaped by a trapezoidal velocity profile over arc length,
// ramping from `min_v_mm_s` at the endpoints up to `default_v_max` on the
// plateau and back down. For paths too short to reach the plateau the
// result is triangular, which is the correct physical behaviour.
// `default_margin` is applied as-is to every point. `accel_mm_s2` is the
// accel/decel used to shape the ramps; it must stay below the downstream
// path controller's slew-rate cap or the profile will be clipped and have
// no effect. `min_v_mm_s` keeps endpoint v_max above 0 so Pure Pursuit
// keeps driving toward its terminal-rotate phase;
std::vector<PathPoint> bcubic_spline(
    const std::vector<PathPoint>& control_pts,
    int    samples_per_segment = 100,
    double default_v_max       = 150.0,
    double default_margin      = 30.0,
    double accel_mm_s2         = 200.0,
    double min_v_mm_s          = 0.0);
