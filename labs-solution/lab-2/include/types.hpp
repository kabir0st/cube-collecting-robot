#pragma once
#include <vector>

// Pose of the lidar (or robot) in world coordinates.
struct Pose2D {
    float x;        // mm
    float y;        // mm
    float theta;    // rad
};

// One wall segment of the reference map.
//   n . p = r is the line equation; (nx, ny) is the unit normal,
//   pointing away from the sensor origin used during calibration.
//   (ax, ay) -> (bx, by) are the segment endpoints in world frame (mm).
struct LineSeg {
    float nx, ny, r;
    float ax, ay, bx, by;
};

// Network ports used by the lidar driver and our process.
//   - Driver listens on CONTROL_PORT for start/stop signals.
//   - We listen on LIDAR_STREAM_PORT for raw lidar packets.
const char* const TCP_IP            = "127.0.0.1";
constexpr int     CONTROL_PORT      = 9887;
constexpr int     LIDAR_STREAM_PORT = 9888;

// One revolution of lidar samples. `angle` is integer degrees [0, 359];
// `distance` is millimetres. Samples with quality<=0 or distance<=0 are
// dropped before reaching this struct.
struct LidarDataFrame {
    std::vector<int> angle;
    std::vector<int> distance;

    LidarDataFrame() {
        angle.reserve(360);
        distance.reserve(360);
    }
};
