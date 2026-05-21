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
void send_state(const Pos2D& pose, const Pose2DCov& cov,
                const Pos2D& goal, const PathPoint& target,
                LidarData& lidar);

}
