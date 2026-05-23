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
        // Hardcoded test sequence: forward 400 mm -> +90° turn + 300 mm ->
        // back home. Entered via start_test().
        TEST,
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
    // Begin the hardcoded TEST sequence: IDLE/DONE/SAFE_STOP -> TEST.
    // First leg is planned on the next step() tick.
    void start_test();
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

    Command step(const Pos2D& pose, const Pose2DCov& cov, Dashboard& dash);

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

    // Failed-approach memory: world-frame position of the box that just
    // timed out / failed to grip. If SEARCH later sees a target inside
    // _failed_target_skip_radius_mm of this point and we are still inside
    // _failed_target_skip_s of the failure, skip it (treat it the same as
    // an at-home box) instead of immediately re-entering APPROACH. This
    // breaks the SEARCH→APPROACH→timeout→SEARCH livelock when a box is
    // wedged or otherwise unreachable.
    bool   _last_failed_target_valid = false;
    double _last_failed_target_x     = 0.0;
    double _last_failed_target_y     = 0.0;
    std::chrono::steady_clock::time_point _last_failed_at;

    // Very-simple search behavior: drive forward _search_forward_target_mm,
    // then rotate 2π in place, repeat. No map waypoints, no spline planning.
    // Obstacle avoidance lives inside FORWARD: while app::obstacle_ahead is
    // true the robot rotates in place to sidestep before resuming forward
    // travel.
    struct SimpleSearch {
        enum class Phase { FORWARD, ROTATE };
        Phase  phase = Phase::FORWARD;
        double forward_traveled_mm  = 0.0;
        double rotation_accumulated = 0.0;
        // Last pose sample used for delta accumulation. Latched on first
        // tick after entry and refreshed every tick thereafter.
        double last_x     = 0.0;
        double last_y     = 0.0;
        double last_theta = 0.0;
        bool   initialized = false;
    };
    SimpleSearch _ssearch;

    // Gripper sequencing inside APPROACH. On grabbable we first defer one
    // tick with v=0 (so the motor has a control cycle to decelerate from
    // chase speed), then fire the smooth-close on the next tick and wait
    // for servo_done(0) (with a 3s watchdog) before handing off to
    // CARRY_HOME — earlier transitions started driving while the gripper
    // was still slewing and the box would slip.
    bool _grip_stop_sent = false;       // tick N: motor told to stop
    bool _grip_fired     = false;       // tick N+1+: smooth close fired
    std::chrono::steady_clock::time_point _grip_fired_at;

    // GOTO state: latched single target pose. Set by go_to(), consumed once
    // on entry to State::GOTO inside _transition() to build the spline.
    // Protected by _m.
    double _goto_x = 0.0;
    double _goto_y = 0.0;
    double _goto_theta = 0.0;
    bool   _goto_pending = false;
    double _goto_v_max_mm_s = 60.0 * ROBOT_SPEED;    // a bit faster than SEARCH; tunable
    double _goto_margin_mm  = 40.0;    // tighter than SEARCH for lab accuracy

    // TEST state: walks the hardcoded leg list defined in controller.cpp.
    // _test_replan_needed is set on entry and after each leg arrival; the
    // next step() tick consumes it to build the spline for the next leg.
    size_t _test_step           = 0;
    bool   _test_replan_needed  = false;
    double _test_v_max_mm_s     = 50.0 * ROBOT_SPEED;
    double _test_margin_mm      = 40.0;

    // Halt-and-localize bookkeeping. When PurePursuit reports arrived for a
    // closed-loop leg (TEST/GOTO), we latch _halting=true, command zero
    // velocity, and wait HALT_MIN_MS..HALT_MAX_MS for σ_xy to settle (Cox
    // folds ~3-5 fresh fixes into the KF while the wheels stand still and
    // add no odometry noise). The Mahalanobis gate then checks the leg
    // target sits inside the post-halt covariance before we advance.
    // _halt_target_{x,y} snapshots the leg target at halt entry so the
    // gate uses the same goal the controller was driving toward.
    bool _halting = false;
    std::chrono::steady_clock::time_point _halt_started;
    double _halt_target_x = 0.0;
    double _halt_target_y = 0.0;

    // Tunables. Conservative defaults; safe to tweak from main if needed.
    double _search_omega        = 0.35 * ROBOT_SPEED;  // rad/s, ~20°/s so camera gets enough frames
    double _search_min_dwell_s  = 0.0;   // engage on sight; failed-target skip handles bounce protection
    double _search_v_forward_mm_s   = 40.0 * ROBOT_SPEED;  // forward speed during the FORWARD leg
    double _search_forward_target_mm = 1000.0;             // 100 cm leg before each look-around rotation
    double _approach_v          = 30.0 * ROBOT_SPEED;  // mm/s, visual-servo cruise
    double _approach_k_omega    = 1.8;   // rad/s per rad of bearing
    double _approach_omega_max  = 1.2 * ROBOT_SPEED;   // clamp on steering rate
    // While vision is dropped during APPROACH we drive toward the last
    // latched target world position. Once the robot is inside this radius
    // of that point and the target is still not visible, give up and
    // fall back to SEARCH (rotate).
    double _dead_reckon_arrival_mm = 100.0;
    double _release_v           = -40.0; // mm/s, backward
    double _release_seconds     = 2.0;   // ~80 mm back-up at 40 mm/s — clears the box from a subsequent in-place rotation, and covers the 1.6s servo OPEN slew with margin
    double _release_hard_timeout_s = 5.0;// give up if box stays stuck on arms
    double _search_timeout_s    = 60.0;
    double _approach_timeout_s  = 20.0;
    double _carry_timeout_s     = 90.0;
    // Hard timeout on TURN_AROUND (now REALIGN-to-home_theta) so a jammed
    // motor doesn't wedge the mission. If we cannot reach the target
    // heading within this many seconds, give up and fall back to SEARCH
    // so the mission can still make progress (or hit its own
    // timeout/budget guard).
    double _turn_around_timeout_s = 8.0;
    // Failed-target avoidance: how close (mm) and how recent (s) a
    // SEARCH detection must be to a previously-failed target before we
    // skip it.
    double _failed_target_skip_radius_mm = 200.0;
    double _failed_target_skip_s         = 30.0;

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
};
