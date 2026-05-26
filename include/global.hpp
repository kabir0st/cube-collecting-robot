#pragma once
#include <atomic>
#include <cmath>

extern double ROBOT_SPEED;

namespace app {
    extern std::atomic<bool> shutdown;
    extern std::atomic<bool> lidar_ready;
    extern std::atomic<bool> lidar_stop;
    extern std::atomic<bool> obstacle_ahead;
    extern std::atomic<double> obstacle_left_mm;
    extern std::atomic<double> obstacle_right_mm;
    extern std::atomic<bool> relocalizing;
    extern std::atomic<bool> cox_first_fix;
    extern std::atomic<bool> motors_ready;
    // Last (v, ω) commanded by loop_control after gating. Published by
    // core.cpp every tick (~25 Hz); read by loop_scan_cox_estimation to
    // detect "I commanded a halt but Cox sees the chassis somewhere
    // else"
    extern std::atomic<double> v_cmd_last_mm_s;
    extern std::atomic<double> w_cmd_last_rad_s;
}
