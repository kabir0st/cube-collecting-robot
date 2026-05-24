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

    // True when loop_control should ignore the lidar `obstacle_ahead`
    // gate for this tick. Fires during:
    //   - APPROACH: the visual servo with reactive omega over snap.obstacles
    //     IS the avoidance strategy; the lidar gate would zero v whenever
    //     the target itself entered the obstacle arc, stalling the grab.
    //   - CARRY_HOME / RELEASE / TURN_AROUND: the closed gripper + held
    //     box sits inside OBSTACLE_DIST_MM and would pin the gate true
    //     on its own payload.
    // SEARCH keeps the gate enabled — there the gripper is open and the
    // forward arc is the right safety net against unseen walls.
    bool obstacle_gate_suppressed() const;

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
    double _home_margin = 50.0;
    // SEARCH "ignore boxes already at home" filter: any detected box whose
    // world position is within this radius of (_home_x, _home_y) is treated
    // as a previously-delivered drop and skipped. Decoupled from
    // _home_margin (the arrival tolerance) because a released box can
    // settle well off the chassis-stop center.
    double _home_skip_radius_mm = 500.0;

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

    // Simple search behavior. Pre-condition (enforced by _enter_search):
    // the robot has been staged to (clamped x, y=0, theta in {0, pi}) so
    // every cycle starts from the home y-line with a known heading.
    // The SEARCH-axis heading is snapshotted ONCE on first-tick init into
    // forward_heading_ref and held for the duration of this SEARCH entry.
    //   1) LOOK_LEFT: rotate CCW until pose.theta reaches ref + π/2.
    //   2) LOOK_RIGHT: rotate CW until pose.theta reaches ref - π/2.
    //   3) FORWARD: realign to ref (hysteresis-guarded), then drive
    //      _search_forward_target_mm along the ref heading. If
    //      app::obstacle_ahead fires, rotate toward whichever half-arc
    //      has more clearance (app::obstacle_{left,right}_mm), bounded
    //      by _search_obstacle_max_rotation_rad; once the obstacle
    //      clears, realign-to-ref kicks in again before resuming.
    // The sweep pattern stays locked to one axis — no per-cycle drift.
    struct SimpleSearch {
        enum class Phase { LOOK_LEFT, LOOK_RIGHT, FORWARD };
        Phase  phase = Phase::LOOK_LEFT;
        double forward_traveled_mm  = 0.0;
        // Bounded obstacle-rotate bookkeeping (FORWARD phase only). The
        // sign is latched on first obstacle tick from side-clearance and
        // stays put until the obstacle clears; the magnitude accumulates
        // |delta theta| so we can abandon the leg after a half-turn rather
        // than spin forever.
        double obstacle_rotated_rad = 0.0;
        double obstacle_dir_sign    = 0.0;   // 0 = not currently dodging
        // Signed rotation accumulated since LOOK_RIGHT entry. Goes
        // negative as we rotate CW; phase exits at -π. LOOK_RIGHT spans
        // ≈ π through the back of the robot, so an absolute-target check
        // (wrap_pi((ref - π/2) - pose)) is +π on entry (antipodal) and
        // would spuriously satisfy err ≥ 0 on tick 0.
        double look_right_rotated_rad = 0.0;
        // Fixed SEARCH-axis heading. Snapshotted on first-tick init
        // (_ssearch.initialized rising edge); held constant across all
        // subsequent LOOK_LEFT/LOOK_RIGHT/FORWARD cycles so the search
        // pattern doesn't walk around the arena.
        double forward_heading_ref  = 0.0;
        // Sticky realign state with hysteresis. Set true after LOOK_RIGHT
        // and after each obstacle dodge clears; cleared only when the
        // heading error sits inside _search_realign_exit_rad for
        // _search_realign_stable_ticks consecutive ticks. Eliminates the
        // R/L/FWD flapping caused by raw cox jitter near the deadband.
        bool   in_realign           = false;
        int    realign_stable_count = 0;
        // Last pose sample used for forward-odometer projection.
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

    // Push-then-grip phase. Latched on the first `grabbable` tick. We
    // drive forward slowly for _approach_push_distance_mm (or until
    // _approach_push_timeout_s) so the target is fully seated between
    // the gripper arms before the close-servo sequence fires. Avoids
    // half-grip captures when the visual gate fires slightly short.
    bool   _push_active = false;
    std::chrono::steady_clock::time_point _push_started_at;
    double _push_start_x = 0.0;
    double _push_start_y = 0.0;

    // Last time vision had a valid target during APPROACH. Reset on entry;
    // refreshed every tick the tracker reports the box. Drives the
    // _approach_vision_lost_timeout_s watchdog. Only touched from the
    // loop_control thread.
    std::chrono::steady_clock::time_point _approach_last_seen_at;

    // World-frame memory of obstacle-color boxes detected by vision.
    // Each Entry is a single-color blob's projected world position, with
    // a last_seen timestamp. step() merges this-frame observations
    // (snap.obstacles) into matching entries within
    // _obstacle_merge_radius_mm of the same color_id, and forgets
    // entries older than _obstacle_forget_s. Single-thread owned by
    // step(); no locking.
    struct ObstacleMemory {
        struct Entry {
            double x_mm = 0.0;
            double y_mm = 0.0;
            int    color_id = -1;
            std::chrono::steady_clock::time_point last_seen;
        };
        std::vector<Entry> entries;
    };
    ObstacleMemory _obstacles;

    // GOTO state: latched single target pose. Set by go_to(), consumed once
    // on entry to State::GOTO inside _transition() to build the spline.
    // Protected by _m.
    double _goto_x = 0.0;
    double _goto_y = 0.0;
    double _goto_theta = 0.0;
    bool   _goto_pending = false;
    // One-shot chain target: when GOTO arrives, transition to this state
    // instead of DONE. Set by _enter_search() to stage before SEARCH;
    // reset to DONE on consumption and on every external go_to() call so
    // the chain can't leak between calls. Protected by _m.
    State  _post_goto_state = State::DONE;
    // Shared spline-leg tunables for any planner that calls `_plan_path_to`
    // (CARRY_HOME, GOTO, TEST). `_planned_v_max_mm_s` is the cruise cap
    // handed to bcubic_spline; `_planned_margin_mm` is the PurePursuit
    // arrival radius. `_home_margin` (configured per mission) stays
    // separate because it lives on a different axis (mission geometry).
    double _planned_v_max_mm_s = 60.0 * ROBOT_SPEED;
    double _planned_margin_mm  = 40.0;

    // TEST state: walks the hardcoded leg list defined in controller.cpp.
    // _test_replan_needed is set on entry and after each leg arrival; the
    // next step() tick consumes it to build the spline for the next leg.
    size_t _test_step           = 0;
    bool   _test_replan_needed  = false;

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
    // _search_omega is NOT scaled with ROBOT_SPEED, same reason as
    // PurePursuit::_omega_max (pure_pursuit.hpp:63): cox runs at ~10 Hz,
    // so faster yaw smears the scan and KF drift accumulates. 0.6 rad/s ≈
    // 34°/s gives cox ~3-4 ticks per 90° sweep half, well inside the
    // motion-compensation envelope.
    double _search_omega        = 0.6;                  // rad/s, fixed
    double _search_v_forward_mm_s   = 50.0 * ROBOT_SPEED;  // forward speed during the FORWARD leg
    double _search_forward_target_mm = 800.0;             // 80 cm leg before each look-around rotation
    // Pre-search staging (see _enter_search). When SEARCH is requested
    // anywhere other than the home line, route through GOTO first to
    // realign to (clamp(x), y=home_y, theta in {0, pi}) so the sweep
    // starts from a consistent geometry. _search_far_threshold_mm picks
    // theta = pi (face back toward home) when we are likely against the
    // far wall; below it we face theta = 0 (open arena ahead).
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
    double _search_stage_theta_tol  = 0.10;             // rad, ~6°
    // Hard cap on cumulative |delta theta| during a single obstacle-clear
    // rotation. Hitting this means the obstacle isn't clearing in this
    // direction — abandon the leg and restart the sweep instead of
    // spinning forever.
    double _search_obstacle_max_rotation_rad = M_PI;
    // Realign-to-ref hysteresis. Enter the realign sub-state when the
    // heading error from forward_heading_ref exceeds `enter_rad`; exit
    // only after error stays below `exit_rad` for `stable_ticks`
    // consecutive ticks. The gap (enter > exit) plus stable_ticks kills
    // the R/L/FWD flap caused by cox heading jitter near the deadband.
    double _search_realign_enter_rad     = 0.15;   // ~9°
    double _search_realign_exit_rad      = 0.05;   // ~3°
    int    _search_realign_stable_ticks  = 3;
    double _approach_v          = 30.0 * ROBOT_SPEED;  // mm/s, visual-servo cruise
    double _approach_k_omega    = 1.8;   // rad/s per rad of bearing
    double _approach_omega_max  = 1.2 * ROBOT_SPEED;   // clamp on steering rate
    // Reactive obstacle avoidance during APPROACH (visual servo only —
    // no path planning). omega_avoid = gain / max(obs.dist, 100) per
    // detected obstacle in front of the target, signed away from the
    // obstacle. Units: rad/s × mm (50 → 0.2 rad/s bias at 250 mm).
    double _approach_repulsion_gain     = 50.0;
    // Push-then-grip parameters. On the first `grabbable` tick we drive
    // forward at _approach_push_v for _approach_push_distance_mm (or
    // until _approach_push_timeout_s) so the target is fully seated in
    // the gripper arms before the close-servo fires.
    double _approach_push_v             = 20.0;    // mm/s, slow
    double _approach_push_distance_mm   = 30.0;
    double _approach_push_timeout_s     = 2.0;
    // While vision is dropped during APPROACH we drive toward the last
    // latched target world position. Once the robot is inside this radius
    // of that point and the target is still not visible, give up and
    // fall back to SEARCH (rotate).
    double _dead_reckon_arrival_mm = 100.0;
    double _release_v           = -50.0; // mm/s, backward
    double _release_seconds     = 4.0;   // ~80 mm back-up at 40 mm/s — clears the box from a subsequent in-place rotation, and covers the 1.6s servo OPEN slew with margin
    double _release_hard_timeout_s = 5.0;// give up if box stays stuck on arms
    double _search_timeout_s    = 60.0;
    double _approach_timeout_s  = 20.0;
    // Watchdog inside APPROACH: if vision has been continuously lost for
    // this many seconds, bail back to SEARCH instead of dead-reckoning
    // indefinitely toward a stale latched target. Independent of the
    // outer _approach_timeout_s (which fires even with vision still up).
    double _approach_vision_lost_timeout_s = 3.0;
    double _carry_timeout_s     = 120.0;
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
    // Obstacle (non-target color) world-frame tracking.
    //  - merge_radius: this-frame observation is merged with an existing
    //    entry of the same color_id within this distance (instead of
    //    appending a duplicate).
    //  - forget_s: entries older than this are dropped on each tick.
    //  - block_radius_mm: SEARCH skips a detected target that sits
    //    within this radius of any remembered obstacle (treats it as
    //    "blocked by obstacle", same skip pattern as at-home boxes).
    double _obstacle_merge_radius_mm     = 200.0;
    double _obstacle_forget_s            = 30.0;
    double _obstacle_block_radius_mm     = 250.0;
    // CARRY_HOME stuck recovery. The robot can push a previously-delivered
    // box into the home wall and stall — no progress, gripper still
    // closed, drag-guard happy. Detection: while in the NORMAL or CURLING
    // sub-phase, if pose displacement stays below
    // _carry_stuck_dist_threshold_mm for _carry_stuck_window_s, declare
    // stuck. Recovery: reverse _carry_recovery_reverse_mm in a straight
    // line (REVERSING phase), then re-plan the spline with the midpoint
    // shifted laterally by _carry_recovery_curl_offset_mm (CURLING
    // phase) so the carried box approaches home off-axis and shoves the
    // old box sideways rather than into the wall. Alternates left then
    // right; after _carry_recovery_max_attempts unsuccessful curls, fall
    // through to SAFE_STOP with the box still gripped.
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
    double _carry_recovery_reverse_v       = 50.0;   // mm/s, magnitude
    double _carry_recovery_reverse_timeout_s = 6.0;  // safety cap
    double _carry_recovery_curl_offset_mm  = 250.0;
    int    _carry_recovery_max_attempts    = 2;

    // State change: runs exit side-effects (clear path, abort controller)
    // on the outgoing state and entry side-effects (plan spline) on the
    // incoming one. Resets _state_entered.
    void _transition(State new_state, const Pos2D& current);

    // Pre-search staging. If the pose is already on the home y-line with
    // theta in {0, pi}, transition directly to SEARCH. Otherwise latch a
    // GOTO target (clamp(x), home_y, stage_theta) with _post_goto_state =
    // SEARCH so the sweep starts from a known geometry. Replaces direct
    // _transition(State::SEARCH, ...) at every external SEARCH entry.
    void _enter_search(const Pos2D& current);

    // Simple 3-control B-cubic spline (start, midpoint, goal) used by
    // CARRY_HOME, GOTO, and TEST. Returns false on an empty path so the
    // caller can route the mission to SAFE_STOP.
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
