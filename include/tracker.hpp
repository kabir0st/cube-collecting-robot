#pragma once

#include <vector>

#include <types.hpp>
#include <dashboard.hpp>

// `target_filter` is the HSV mask for the box the mission should pick up.
// `obstacle_filters` is the (possibly empty) list of HSV masks for boxes
// the mission should avoid / not push around. The tracker emits a single
// TrackerResult: target fields drive the visual servo as before, and a
// per-frame obstacles vector reports the largest blob of each obstacle
// color (above MIN_OBSTACLE_AREA_PX). `cfg.focal_length_px` drives the
// shared pinhole distance estimate.
void loop_vision_tracking(Dashboard& dash,
                          FilterResult target_filter,
                          std::vector<FilterResult> obstacle_filters,
                          CameraConfig cfg,
                          TrackerResult& result);
