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
    // Set true while loop_scan_cox_estimation is running a relocalize
    // grid sweep (cox returned an untrusted estimate). loop_control reads
    // this and halts the motors for the duration of the sweep so we don't
    // drive blind while the KF is being re-pinned.
    extern std::atomic<bool> relocalizing;
}
