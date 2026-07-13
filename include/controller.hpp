#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <utility>
#include <vector>
#include <dashboard.hpp>
#include <global.hpp>
#include <pure_pursuit.hpp>
#include <types.hpp>

// our main state and behaviour logic
class StateMachine {
public:
    enum class State {
        IDLE,
        SEARCH,
        APPROACH,
        CARRY_HOME,
        RELEASE,
        TURN_AROUND,
        DONE,
        SAFE_STOP,
        // for testing to goto specific point
        GOTO,
        // Hardcoded test sequence
        // forward 400 mm -> +90° turn + 300 mm
        TEST,
    };

    // contorl motor
    struct Command {
        enum class Mode {
            STOP,
            DIRECT,
            PATH_FOLLOW,
        };
        Mode   mode         = Mode::STOP;
        double v_mm_s       = 0.0;
        double omega_rad_s  = 0.0;
    };

    StateMachine(TrackerResult& vision,
                 PurePursuitController& controller);

    void start();
    void start_test();
    void abort();

    void configure(double home_x, double home_y, double home_theta,
                   double home_margin, double arm_length);

    // for testing to go to one point
    void go_to(double x, double y, double theta);
    // overriding to use it during replanning
    void go_to(const Pos2D& target);

    bool goto_arrived() const;

    Command step(const Pos2D& pose, const Pose2DCov& cov, Dashboard& dash);

    State state() const;
    static const char* state_name(State s);

    // for moving ahead even though there might something ahead of us 
    bool obstacle_gate_suppressed() const;

private:
    TrackerResult&         _vision;
    PurePursuitController& _controller;

    std::atomic<State> _state{State::IDLE};
    mutable std::mutex _m;
    std::chrono::steady_clock::time_point _state_entered;

    // home data
    double _home_x      = 0.0;
    double _home_y      = 0.0;
    double _home_theta  = 0.0;
    double _home_margin = 50.0;
    
    // ignore the boxes that are here
    double _home_skip_radius_mm = 500.0;

    double _arm_length_mm = 120.0;

    // Retry budget so APPROACH timeout does not bounce back to SEARCH
    // forever when the box is unreachable.
    int _approach_attempts     = 0;
    int _max_approach_attempts = 5;

    // Set the first time we skip an at-home detection cleared
    // when we either see no box, see a non-at-home box, or re-enter SEARCH.
    bool _search_skip_streak = false;

    // to pursue target's last known postion even though we
    // might have lost the target
    bool   _last_target_valid = false;
    double _last_target_x     = 0.0;
    double _last_target_y     = 0.0;

    // if any of the box is unreachable, (set after approach budget is 
    // exhausted we ignore it so we don't stuck on a cycle)
    bool   _last_failed_target_valid = false;
    double _last_failed_target_x     = 0.0;
    double _last_failed_target_y     = 0.0;
    std::chrono::steady_clock::time_point _last_failed_at;

    struct SimpleSearch {
        enum class Phase { LOOK_LEFT, LOOK_RIGHT, FORWARD };
        Phase  phase = Phase::LOOK_LEFT;
        double forward_traveled_mm  = 0.0;
        // check how much we moved to avoid obstacle 
        double obstacle_rotated_rad = 0.0;
        // 0 = not currently dodging
        double obstacle_dir_sign    = 0.0;   

        double look_right_rotated_rad = 0.0;
        // Fixed SEARCH-axis heading. 
        double forward_heading_ref  = 0.0;
        // after rotating for search the robot drifts
        // we need to realign back to search axis
        bool   in_realign           = false;
        int    realign_stable_count = 0;
        // calculate for accumulation and check
        double last_x     = 0.0;
        double last_y     = 0.0;
        double last_theta = 0.0;

        bool   initialized = false;
    };
    SimpleSearch _ssearch;

    bool _grip_stop_sent = false;      
    bool _grip_fired     = false;
    std::chrono::steady_clock::time_point _grip_fired_at;

    
    bool   _push_active = false;
    std::chrono::steady_clock::time_point _push_started_at;
    double _push_start_x = 0.0;
    double _push_start_y = 0.0;

    // tracking vision lost time
    std::chrono::steady_clock::time_point _approach_last_seen_at;

    double _goto_x = 0.0;
    double _goto_y = 0.0;
    double _goto_theta = 0.0;
    bool   _goto_pending = false;

    State  _post_goto_state = State::DONE;

    double _planned_v_max_mm_s = 60.0 * ROBOT_SPEED;
    double _planned_margin_mm  = 40.0;

    size_t _test_step           = 0;
    bool   _test_replan_needed  = false;

    bool _halting = false;
    std::chrono::steady_clock::time_point _halt_started;
    double _halt_target_x = 0.0;
    double _halt_target_y = 0.0;

    double _search_omega        = 0.4;
    double _search_v_forward_mm_s   = 50.0 * ROBOT_SPEED;
    double _search_forward_target_mm = 800.0;

    double _search_far_threshold_mm = 3400.0;
    double _search_stage_x_min_mm   = 300.0;
    // Staging clamp matches _search_x_far_turn_mm so _enter_search
    // never plants the robot past the in-sweep turn-back boundary.
    double _search_stage_x_max_mm   = 2000.0;
    // In-sweep turn-back. Once pose.x crosses this while driving the
    // FORWARD leg with the heading pointing into the arena (+x), flip
    // forward_heading_ref by π and restart the sweep so the next legs
    // walk back toward home. No effect when already facing home.
    double _search_x_far_turn_mm    = 2000.0;
    double _search_stage_theta_tol  = 0.10;
    double _search_obstacle_max_rotation_rad = M_PI;

    // if off by this, we entry realign
    double _search_realign_enter_rad     = 0.15;   // ~9°
    double _search_realign_exit_rad      = 0.05;   // ~3°
    int    _search_realign_stable_ticks  = 3;
    double _approach_v          = 30.0 * ROBOT_SPEED;
    double _approach_k_omega    = 1.2;
    double _approach_omega_max  = 1.2;
    double _approach_push_v             = 20.0;    // mm/s, slow
    double _approach_push_distance_mm   = 30.0;
    double _approach_push_timeout_s     = 2.0;

    double _dead_reckon_arrival_mm = 100.0;
    double _release_v           = -50.0; // mm/s, backward
    double _release_seconds     = 4.0; 
    double _release_hard_timeout_s = 5.0;
    double _search_timeout_s    = 60.0;
    double _approach_timeout_s  = 20.0;

    double _approach_vision_lost_timeout_s = 3.0;
    double _carry_timeout_s     = 120.0;
    
    double _turn_around_timeout_s = 8.0;
    // How close SEARCH detection must be to a previously-failed 
    // target before we skip it.
    double _failed_target_skip_radius_mm = 200.0;
    double _failed_target_skip_s         = 30.0;



    // CARRY_HOME stuck recovery. 
    enum class CarryPhase { NORMAL, REVERSING, CURLING };
    CarryPhase _carry_phase = CarryPhase::NORMAL;
    std::chrono::steady_clock::time_point _carry_phase_entered;
    Pos2D  _stuck_anchor_pose{};
    std::chrono::steady_clock::time_point _stuck_anchor_at;
    Pos2D  _reverse_start_pose{};
    int    _carry_recovery_attempts        = 0;
    double _carry_stuck_dist_threshold_mm  = 30.0;
    double _carry_stuck_window_s           = 3.0;
    double _carry_recovery_reverse_mm      = 200.0;
    double _carry_recovery_reverse_v       = 50.0;
    double _carry_recovery_reverse_timeout_s = 6.0;
    double _carry_recovery_curl_offset_mm  = 250.0;
    int    _carry_recovery_max_attempts    = 2;

    // State change: runs exit side-effects (clear path, abort controller)
    // on the outgoing state and entry side-effects (plan spline) on the
    // incoming one. Resets _state_entered.
    void _transition(State new_state, const Pos2D& current);

    // Pre-search staging
    void _enter_search(const Pos2D& current);

    bool _plan_path_to(const Pos2D& current,
                       double gx, double gy, double gtheta,
                       double margin_mm, double v_max_mm_s);

    // Same as _plan_path_to but shifts the midpoint control point
    // perpendicular to the start->goal line by `lateral_offset_mm`.
    // Positive = left of the approach direction (i.e. +Y in the local
    // body frame of the start->goal vector). Used by CARRY_HOME stuck
    // recovery to curl into home off-axis instead of pushing straight in.
    bool _plan_path_via_offset(const Pos2D& current,
                               double gx, double gy, double gtheta,
                               double margin_mm, double v_max_mm_s,
                               double lateral_offset_mm);
};
