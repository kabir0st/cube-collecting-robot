#pragma once

#include <types.hpp>
#include <dashboard.hpp>

// `target_filter` is the HSV mask for the box the mission should pick up.
// `cfg.focal_length_px` drives the pinhole distance estimate.
void loop_vision_tracking(Dashboard& dash,
                          FilterResult target_filter,
                          CameraConfig cfg,
                          TrackerResult& result);
