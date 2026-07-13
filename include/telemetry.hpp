#pragma once

#include <string>
#include <vector>

#include <types.hpp>

namespace telemetry {

void init(const std::string& host, int port, bool enabled,
          int scan_subsample);
void shutdown();

void send_map(const std::vector<LineSeg>& walls);
void send_path(const std::vector<PathPoint>& path);
void send_state(const Pos2D& pose, const Pose2DCov& cov,
                const Pos2D& goal,
                const PathPoint& target, bool target_valid,
                LidarData& lidar);

}
