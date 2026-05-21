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

// the main logic of the project
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
        // Bench/lab mode: drive to a single (x, y, theta) via the spline +
        // PurePursuit pipeline, then latch DONE. Orthogonal to the
        // SEARCH-based mission; entered via go_to(), not start().
        GOTO,
    };

    // What loop_control should do with the motors this tick.
    struct Command {
        enum class Mode {
            STOP,         // zero velocity; controller aborted; follower cleared
            DIRECT,       // use (v_mm_s, omega_rad_s) below; bypass controller
            PATH_FOLLOW,  // run the path-following controller
        };
        Mode   mode         = Mode::STOP;
        double v_mm_s       = 0.0;
        double omega_rad_s  = 0.0;
    };

    StateMachine(TrackerResult& vision,
                 PurePursuitController& controller);

    // Begin the mission: IDLE -> SEARCH. No-op from any other state.
    void start();
    void abort();

    // Load mission geometry from config (overrides constructor defaults).
    // Must be called from the owner thread before start() or go_to().
    void configure(double home_x, double home_y, double home_theta,
                   double home_margin, double arm_length);

    // Lab/bench entry: drive to (x, y, theta) via a B-cubic spline +
    // PurePursuit (TERMINAL_ROTATE enforces the final heading). Allowed from
    // IDLE / DONE / SAFE_STOP / GOTO. Latches the target and transitions to
    // GOTO; the next step() builds the spline and starts following.
    void go_to(double x, double y, double theta);
    void go_to(const Pos2D& target);
    // True once the controller has arrived at the latched goal pose (i.e.
    // PurePursuit has reached DONE). Useful for outer loops to know when
    // the goto completed.
    bool goto_arrived() const;

    Command step(const Pos2D& pose, Dashboard& dash);

    State state() const;
    static const char* state_name(State s);

private:
    // Orchestrated subsystems (refs; lifetimes owned by main).
    TrackerResult&         _vision;
    PurePursuitController& _controller;

    // State + per-state bookkeeping. _m protects the time_point and home
    // pos
    std::atomic<State> _state{State::IDLE};
    mutable std::mutex _m;
    std::chrono::steady_clock::time_point _state_entered;
    std::chrono::steady_clock::time_point _last_replan_at;

    // Mission target.
    double _home_x      = 0.0;
    double _home_y      = 0.0;
    double _home_theta  = 0.0;
    double _home_margin = 100.0;

    // Geometry of the gripper. Box position when captured =
    //   robot.xy + _arm_length_mm * (cos theta, sin theta).
    double _arm_length_mm = 120.0;

    // Retry budget so APPROACH timeout does not bounce back to SEARCH
    // forever when the box is unreachable.
    int _approach_attempts     = 0;
    int _max_approach_attempts = 5;

    // Rising-edge log gate for the "skip already-delivered box in SEARCH"
    // filter. Set the first time we skip an at-home detection; cleared
    // when we either see no box, see a non-at-home box, or re-enter SEARCH.
    // Only touched from the loop_control thread, so no lock needed.
    bool _search_skip_streak = false;

    // Last known world-frame target position, latched during APPROACH on
    // every tick where the tracker reports a valid distance. When vision
    // drops out we dead-reckon toward this point instead of immediately
    // bouncing to SEARCH (which would spin in place and re-lose the
    // direction we just had). Cleared on entry to SEARCH. Only touched
    // from the loop_control thread.
    bool   _last_target_valid = false;
    double _last_target_x     = 0.0;
    double _last_target_y     = 0.0;

    bool   _search_path_active = false;
    size_t _search_last_wp_idx = static_cast<size_t>(-1);

    // Adaptive-subdivision search plan. Regenerated on every entry to SEARCH
    // from the arena bounding box published by the calibrations module.
    struct SearchPlan {
        std::vector<std::pair<double, double>> waypoints;  // world-frame (x, y) mm
        size_t idx = 0;
        bool   at_waypoint = false;          // false = travelling, true = rotating
        double rotation_start_theta = 0.0;
        double accumulated_rotation = 0.0;
        bool   valid = false;
    };
    SearchPlan _search_plan;

    // Gripper sequencing inside APPROACH. On grabbable we first defer one
    // tick with v=0 (so the motor has a control cycle to decelerate from
    // chase speed), then fire the smooth-close on the next tick and wait
    // for `captured` to flip.
    bool _grip_stop_sent = false;       // tick N: motor told to stop
    bool _grip_fired     = false;       // tick N+1+: smooth close fired
    std::chrono::steady_clock::time_point _grip_fired_at;

    // 180° pivot bookkeeping for TURN_AROUND.
    bool   _turn_start_set = false;
    double _turn_start_theta = 0.0;
    double _turn_accumulated = 0.0;

    // GOTO state: latched single target pose. Set by go_to(), consumed once
    // on entry to State::GOTO inside _transition() to build the spline.
    // Protected by _m.
    double _goto_x = 0.0;
    double _goto_y = 0.0;
    double _goto_theta = 0.0;
    bool   _goto_pending = false;
    double _goto_v_max_mm_s = 60.0 * ROBOT_SPEED;    // a bit faster than SEARCH; tunable
    double _goto_margin_mm  = 40.0;    // tighter than SEARCH for lab accuracy

    // Tunables. Conservative defaults; safe to tweak from main if needed.
    double _search_omega        = 0.35 * ROBOT_SPEED;  // rad/s, ~20°/s so camera gets enough frames
    double _search_min_dwell_s  = 2.0;   // bounce guard between failed approaches
    double _search_v_max_mm_s   = 40.0 * ROBOT_SPEED;  // travel speed between waypoints
    double _waypoint_margin_mm  = 80.0;  // arrival radius + wall standoff
    double _box_detection_range_mm = 700.0;  // drives subdivision math
    double _grip_settle_s       = 1.0;   // wait for `captured` after closing servo (vision needs ~10 frames to debounce)
    double _approach_v          = 30.0 * ROBOT_SPEED;  // mm/s, visual-servo cruise
    double _approach_k_omega    = 1.8;   // rad/s per rad of bearing
    double _approach_omega_max  = 1.2 * ROBOT_SPEED;   // clamp on steering rate
    // While vision is dropped during APPROACH we drive toward the last
    // latched target world position. Once the robot is inside this radius
    // of that point and the target is still not visible, give up and
    // fall back to SEARCH (rotate).
    double _dead_reckon_arrival_mm = 100.0;
    double _release_v           = -40.0; // mm/s, backward
    double _release_seconds     = 1.25;  // ~50 mm back-up at 40 mm/s
    double _release_hard_timeout_s = 5.0;// give up if box stays stuck on arms
    double _search_timeout_s    = 60.0;
    double _approach_timeout_s  = 20.0;
    double _carry_timeout_s     = 90.0;
    double _carry_replan_s      = 5.0;   // re-plan spline this often during CARRY_HOME

    // State change: runs exit side-effects (clear path, abort controller)
    // on the outgoing state and entry side-effects (plan spline) on the
    // incoming one. Resets _state_entered.
    void _transition(State new_state, const Pos2D& current);

    // CARRY_HOME entry: build a 3-point B-cubic spline (current → midpoint
    // → home) and load it into the controller. Returns false if the
    // spline generator returned an empty path (caller should route the
    // mission to SAFE_STOP rather than silently bouncing through CARRY_HOME).
    bool _plan_path_home(const Pos2D& current);

    bool _plan_path_to(const Pos2D& current,
                       double gx, double gy, double gtheta,
                       double margin_mm, double v_max_mm_s);

    // Build the adaptive-subdivision waypoint list from the calibrations
    // module's published ArenaBounds. Sets _search_plan.valid = false if the
    // bounds are not yet ready.
    void _build_search_plan(const Pos2D& pose);
};
