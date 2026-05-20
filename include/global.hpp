#pragma once
#include <atomic>

// Global speed multiplier applied to every linear/angular velocity cap and
// acceleration limit. 1.0 = the tuned baseline. Bump up to make the robot
// faster (driving + turning + ramp rates all scale together so cruise time
// stays the same shape); drop below 1.0 to slow everything down.
constexpr double ROBOT_SPEED = 1.0;

namespace app {
    extern std::atomic<bool> shutdown;
    extern std::atomic<bool> lidar_ready;
    extern std::atomic<bool> lidar_stop;
    extern std::atomic<bool> obstacle_ahead;
}
