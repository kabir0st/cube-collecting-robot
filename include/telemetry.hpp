#pragma once

#include <string>
#include <vector>

#include <types.hpp>

// Minimal best-effort UDP telemetry sender. Newline-delimited JSON, one
// packet per message. Sends are no-ops when disabled or before init().
// Thread-safe; callers don't need to coordinate.
namespace telemetry {

void init(const std::string& host, int port, bool enabled,
          int scan_subsample);
void shutdown();

void send_map(const std::vector<LineSeg>& walls);
void send_path(const std::vector<PathPoint>& path);
// `target_valid` distinguishes "no path loaded" (false; receiver should hide
// the target marker) from "path active, target is at world origin"
// (true; receiver should draw the marker at (0, 0)).
void send_state(const Pos2D& pose, const Pose2DCov& cov,
                const Pos2D& goal,
                const PathPoint& target, bool target_valid,
                LidarData& lidar);

}
