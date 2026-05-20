#pragma once

#include <types.hpp>
#include <dashboard.hpp>

// `filter` is the HSV mask produced by CalibrateCamera::start_filter_calibration().
// `cfg.focal_length_px` and `cfg.box_width_mm` drive the distance estimate.
void loop_vision_tracking(Dashboard& dash,
                          FilterResult filter,
                          CameraConfig cfg,
                          TrackerResult& result);
