#pragma once
#include <vector>

#include <types.hpp>

// Apply a trapezoidal velocity profile to a dense path, modifying each
// point's `v_max` in place.
//
// Along the cumulative arc length s in [0, S_total]:
//   v_up   = sqrt(2 * a * s)         (limited by how fast you can ramp up)
//   v_down = sqrt(2 * a * (S - s))   (limited by how fast you can brake)
//   v_max  = max(min_v_mm_s, min(v_cruise, v_up, v_down))
//
// For short paths the cruise plateau may not be reached — the result is a
// triangular profile, which is the correct physical behaviour.
//
// `min_v_mm_s` is a floor that keeps the endpoints from collapsing to 0.
// The textbook trapezoidal profile gives v_max = 0 at s = 0 and s = S_total,
// but a downstream Pure Pursuit controller reading path.back().v_max sees
// that 0, commands v = 0, and never reaches its terminal-rotate / arrived
// state — the robot freezes a few cm short of the goal. A small floor
// (~10-30 mm/s) keeps the controller crawling into its margin so the
// terminal phase can take over. Pass 0 to disable.
void apply_trap_profile(std::vector<PathPoint>& path,
                        double v_cruise_mm_s,
                        double accel_mm_s2,
                        double min_v_mm_s = 0.0);
