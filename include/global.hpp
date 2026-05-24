#pragma once
#include <atomic>
#include <cmath>

// Angle from the robot's +x (forward) to the LIDAR's 0° scan direction,
// measured CCW. Used both by the scan-matcher (cox.cpp) and by the wall
// extractor (find_walls in calibrations.cpp) so the reference map and the
// per-tick scans live in the same body frame.
//   0      : LiDAR's 0° already points forward
//   -π/2   : LiDAR's 0° points to the robot's right (LiDAR rotated 90° CW)
//   +π/2   : LiDAR's 0° points to the robot's left  (LiDAR rotated 90° CCW)
//    π     : LiDAR's 0° points backward
// Adjust this if pushing the robot forward does *not* make Cox.X increase.
//
// Verified empirically: with 0, push forward -> Cox.X increases, push right
// -> Cox.Y decreases, CCW rotation -> Cox.theta increases (standard right-
// handed body frame, matching the odometry/encoder convention so KF fusion
// is consistent). Previously -π/2 left Cox 90° CW-rotated from odometry --
// fusion still "worked" but the two estimates fought each other and the
// dashboard X/Y didn't match the robot's actual heading.
constexpr double LIDAR_MOUNT_OFFSET_RAD = 0.0;

// Global speed multiplier applied to every linear/angular velocity cap and
// acceleration limit. 1.0 = the original tuned baseline. Bump up to make
// the robot faster (driving + turning + ramp rates all scale together so
// cruise time stays the same shape); drop below 1.0 to slow everything
// down.
//
// Loaded from `robot.speed` in config.yml at startup (see main.cpp). The
// initial value below is only the boot default; it's overwritten before
// any motor / controller object is constructed, so member initializers
// that reference ROBOT_SPEED pick up the configured value automatically.
//
// Soft ceiling for clean scaling depends on chassis hardware caps in
// motor.cpp. With the current ~504 mm/s / ~5.45 rad/s caps, ROBOT_SPEED
// up to ~3.6 stays clean; above that the motor silently clamps omega on
// sharp turns. See docs/coordinate-frames.md for the full table.
extern double ROBOT_SPEED;

namespace app {
    extern std::atomic<bool> shutdown;
    extern std::atomic<bool> lidar_ready;
    extern std::atomic<bool> lidar_stop;
    extern std::atomic<bool> obstacle_ahead;
    // Per-side forward-arc minimum distance (mm), measured inside the same
    // OBSTACLE_ARC_HALF_DEG cone as obstacle_ahead. SEARCH uses these to
    // pick which way to rotate when clearing an obstacle (turn toward the
    // freer side). +inf when no beam in that half-arc passed the gripper
    // mask this frame. Left = body-frame +Y, right = body-frame -Y.
    extern std::atomic<double> obstacle_left_mm;
    extern std::atomic<double> obstacle_right_mm;
    // Set true while loop_scan_cox_estimation is running a relocalize
    // grid sweep (cox returned an untrusted estimate). loop_control reads
    // this and halts the motors for the duration of the sweep so we don't
    // drive blind while the KF is being re-pinned.
    extern std::atomic<bool> relocalizing;
    // Latched true the first time cox produces a fix that passes
    // cox_can_fuse() and is folded into the KF. loop_control blocks the
    // mission start on this so we never drive on the seeded (0,0,0) pose.
    extern std::atomic<bool> cox_first_fix;
    // Latched true once loop_control has called motor.start() and given
    // the chassis a moment to settle. loop_localization blocks lidar
    // frame collection on this so the reference map references the
    // post-startup pose. Without it, an SPI-handshake wheel pulse can
    // displace the body AFTER the map is built, leaving the cox seed at
    // (0,0,0) outside its first-fix basin of convergence.
    extern std::atomic<bool> motors_ready;
}
