#include <controller.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <string>

#include <calibrations.hpp>
#include <global.hpp>
#include <loggers.hpp>
#include <servo.h>
#include <utils.hpp>
#include <types.hpp>
#include <spline.hpp>

namespace {

inline double seconds_since(std::chrono::steady_clock::time_point ts) {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now() - ts).count();
}

inline double clamp_d(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

inline void polar_to_world(const Pos2D& pose,
                           double bearing_rad, double distance_mm,
                           double& out_x, double& out_y) {
    const double world_bearing = pose.theta + bearing_rad;
    out_x = pose.x + distance_mm * std::cos(world_bearing);
    out_y = pose.y + distance_mm * std::sin(world_bearing);
}

// Hardcoded TEST sequence: forward 400 mm in +x, then 90° CCW + 300 mm in
// +y, then back to (0,0,0). Each entry is (x, y, theta) in the
// localization frame; theta is the desired heading at the end of that leg
// (PurePursuit's TERMINAL_ROTATE enforces it).
struct TestWaypoint { double x, y, theta; };
static const TestWaypoint TEST_WAYPOINTS[] = {
    {400.0,   0.0, 0.0},
    {400.0, 300.0, M_PI / 2.0},
    {  0.0,   0.0, 0.0},
};
static const size_t TEST_WAYPOINT_COUNT =
    sizeof(TEST_WAYPOINTS) / sizeof(TEST_WAYPOINTS[0]);

// Halt-and-localize / Mahalanobis-gate tunables. See _halting field
// docstring in controller.hpp for the overall flow. HALT_MIN_MS gives
// Cox at least one or two 100 ms scan cycles to fold in; HALT_MAX_MS
// caps the wait so a sustained-bad Cox can't deadlock the mission.
// MAHAL_GATE_CHI2 = 5.99 is the χ² 95 % cut-off for 2 DoF (x, y);
// heading is intentionally outside the gate because angle-wrap makes
// the 3-DoF form brittle and PurePursuit's TERMINAL_ROTATE already
// enforces final heading.
constexpr int    HALT_MIN_MS             = 300;
constexpr int    HALT_MAX_MS             = 700;
constexpr double HALT_SIGMA_XY_TARGET_MM = 30.0;
constexpr double MAHAL_GATE_CHI2         = 5.99;
// Additive (x, y) noise floor on the Mahalanobis gate. Cox in a
// well-constrained arena (e.g. 4-wall) reports sub-mm sigma — that
// reflects lidar fit quality, not the robot's true post-arrival pose
// uncertainty, which is dominated by PurePursuit's `_planned_margin_mm`
// (40 mm) arrival tolerance and wheel-slip terminal error. Without this
// floor every TEST leg trips the χ² gate on a few-cm residual.
constexpr double HALT_GATE_POS_NOISE_VAR_MM2 = 40.0 * 40.0;

// Squared Mahalanobis distance from `pose` (x, y) to (tx, ty) under the
// xy-block of `cov`, with HALT_GATE_POS_NOISE_VAR_MM2 added to the
// diagonal so the gate measures residual vs. expected arrival accuracy
// rather than vs. lidar fit quality. Returns +inf when the block is
// degenerate so the gate treats it as "wildly uncertain" rather than
// dividing by zero.
inline double mahalanobis2_xy(const Pos2D& pose, const Pose2DCov& cov,
                              double tx, double ty) {
    const double dx  = tx - pose.x;
    const double dy  = ty - pose.y;
    const double xx  = cov.xx + HALT_GATE_POS_NOISE_VAR_MM2;
    const double yy  = cov.yy + HALT_GATE_POS_NOISE_VAR_MM2;
    const double det = xx * yy - cov.xy * cov.xy;
    if (det <= 0.0) return std::numeric_limits<double>::infinity();
    return (yy * dx * dx
            - 2.0 * cov.xy * dx * dy
            + xx * dy * dy) / det;
}

}  // namespace

const char* StateMachine::state_name(State s) {
    switch (s) {
        case State::IDLE:        return "IDLE";
        case State::SEARCH:      return "SEARCH";
        case State::APPROACH:    return "APPROACH";
        case State::CARRY_HOME:  return "CARRY_HOME";
        case State::RELEASE:     return "RELEASE";
        case State::TURN_AROUND: return "TURN_AROUND";
        case State::DONE:        return "DONE";
        case State::SAFE_STOP:   return "SAFE_STOP";
        case State::GOTO:        return "GOTO";
        case State::TEST:        return "TEST";
    }
    return "?";
}

StateMachine::StateMachine(TrackerResult& vision,
                           PurePursuitController& controller)
    : _vision(vision), _controller(controller) {
    _state_entered = std::chrono::steady_clock::now();
}

void StateMachine::configure(double home_x, double home_y, double home_theta,
                             double home_margin, double arm_length) {
    {
        std::lock_guard<std::mutex> lk(_m);
        _home_x        = home_x;
        _home_y        = home_y;
        _home_theta    = home_theta;
        _home_margin   = home_margin;
        _arm_length_mm = arm_length;
    }
    LOG("Mission") << "configure(): home=(" << home_x << ", " << home_y
                   << ", " << home_theta << " rad), margin=" << home_margin
                   << " mm, arm=" << arm_length << " mm";
}

void StateMachine::start() {
    // Allow start() from any terminal state (IDLE / DONE / SAFE_STOP) so the
    // mission can be re-run without restarting the process.
    std::lock_guard<std::mutex> lk(_m);
    const State now = _state.load();
    const bool terminal = (now == State::IDLE   ||
                           now == State::DONE   ||
                           now == State::SAFE_STOP);
    if (!terminal) {
        LOG("Mission") << "start() ignored; state = " << state_name(now);
        return;
    }
    _state.store(State::SEARCH);
    _state_entered = std::chrono::steady_clock::now();
    _approach_attempts = 0;
    // Fresh mission: drop any stale failed-target avoidance memory so the
    // operator doesn't have to wait _failed_target_skip_s for the next
    // mission to engage a box near the previous failure.
    _last_failed_target_valid = false;
    LOG("Mission") << "start(): " << state_name(now) << " -> SEARCH";
}

void StateMachine::start_test() {
    std::lock_guard<std::mutex> lk(_m);
    const State now = _state.load();
    const bool terminal = (now == State::IDLE   ||
                           now == State::DONE   ||
                           now == State::SAFE_STOP);
    if (!terminal) {
        LOG("Mission") << "start_test() ignored; state = " << state_name(now);
        return;
    }
    _test_step          = 0;
    _test_replan_needed = true;
    _halting            = false;
    _state.store(State::TEST);
    _state_entered = std::chrono::steady_clock::now();
    LOG("Mission") << "start_test(): " << state_name(now) << " -> TEST ("
                   << TEST_WAYPOINT_COUNT << " legs)";
}


void StateMachine::go_to(double x, double y, double theta) {
    // Latch the target under the lock so _transition()'s entry side-effect
    // sees a consistent goal even if go_to() races with a step() tick.
    Pos2D dummy_current{};  // _transition derives the spline from the live
                            // pose passed into step(); we only need to flip
                            // the state here. The actual plan is built on
                            // the first step() tick after the transition.
    bool replaced_pending = false;
    double prev_x = 0.0, prev_y = 0.0, prev_th = 0.0;
    {
        std::lock_guard<std::mutex> lk(_m);
        const State now = _state.load();
        const bool allowed = (now == State::IDLE ||
                              now == State::DONE ||
                              now == State::SAFE_STOP ||
                              now == State::GOTO);
        if (!allowed) {
            LOG("Mission") << "go_to() ignored; state = " << state_name(now);
            return;
        }
        if (_goto_pending) {
            replaced_pending = true;
            prev_x = _goto_x; prev_y = _goto_y; prev_th = _goto_theta;
        }
        _goto_x = x;
        _goto_y = y;
        _goto_theta = theta;
        _goto_pending = true;
        _post_goto_state = State::DONE;   // external GOTO never chains
        _state.store(State::GOTO);
        _state_entered = std::chrono::steady_clock::now();
    }
    if (replaced_pending) {
        LOG("Mission") << "go_to(): displaced un-consumed target ("
                       << prev_x << ", " << prev_y << ", " << prev_th << ")";
    }
    LOG("Mission") << "go_to(" << x << ", " << y << ", "
                   << theta << ") latched";
}

void StateMachine::go_to(const Pos2D& target) {
    go_to(target.x, target.y, target.theta);
}

bool StateMachine::goto_arrived() const {
    const State s = _state.load();
    if (s != State::GOTO && s != State::DONE) return false;
    return _controller.has_arrived();
}

void StateMachine::abort() {
    // Side-effects first so they take effect even if the lock is contended.
    _controller.abort();
    {
        std::lock_guard<std::mutex> lk(_m);
        _state.store(State::SAFE_STOP);
        _state_entered = std::chrono::steady_clock::now();
    }
    LOG("Mission") << "abort(): -> SAFE_STOP";
}

StateMachine::State StateMachine::state() const {
    return _state.load();
}

bool StateMachine::obstacle_gate_suppressed() const {
    const State s = _state.load();
    // CARRY_HOME deliberately NOT suppressed — the lidar mount sits
    // above the boxes, so the held payload doesn't pin the gate; walls
    // do, and we want forward motion to stop instead of pushing through.
    // The CARRY_HOME stuck-recovery branch handles the resulting v=0
    // by reversing + re-planning around the wall.
    return s == State::APPROACH       // visual-servo handles avoidance
        || s == State::RELEASE        // backing up; gate is no-op anyway
        || s == State::TURN_AROUND;   // in-place rotation; gate is no-op
}

void StateMachine::_transition(State new_state, const Pos2D& current) {
    const State old_state = _state.load();

    if (old_state == new_state) return;
    // Any state change clears halt-and-localize bookkeeping so an aborted
    // halt mid-state can't masquerade as a halt belonging to the next.
    _halting = false;
    // Mission-accomplished log: only on the actual completion edge
    // CARRY_HOME -> RELEASE. Used to fire on every CARRY_HOME or SEARCH
    // exit (timeouts, engagement transitions) and produced misleading logs.
    if (old_state == State::CARRY_HOME && new_state == State::RELEASE) {
        LOG("Mission") << "Carried Home! Mission Accomplished";
    }
    // Always abort the path-follower when leaving any state that uses a
    // path. SEARCH and APPROACH use DIRECT mode but may still have a
    // stale path loaded from a prior CARRY_HOME / TURN_AROUND. Earlier
    // this only fired for CARRY_HOME or SEARCH exits; TURN_AROUND -> SEARCH
    // and GOTO -> DONE leaked stale paths into the dashboard target overlay.
    if (old_state == State::CARRY_HOME ||
        old_state == State::TURN_AROUND ||
        old_state == State::GOTO ||
        old_state == State::TEST) {
        _controller.abort();
    }

    if (new_state == State::CARRY_HOME) {
        double hx_l, hy_l, hm_l;
        {
            std::lock_guard<std::mutex> lk(_m);
            hx_l = _home_x; hy_l = _home_y; hm_l = _home_margin;
        }
        const double tan_to_goal =
            std::atan2(hy_l - current.y, hx_l - current.x);
        if (!_plan_path_to(current, hx_l, hy_l, tan_to_goal,
                           hm_l, _planned_v_max_mm_s)) {
            // Don't SAFE_STOP — leave _controller path-less; step()'s
            // CARRY_HOME body retries the plan each tick.
            LOG("Mission")
                << "CARRY_HOME entry plan failed; will retry from step()";
        }
    }

    if (new_state == State::APPROACH) {
        // Each approach starts from a fresh grip cycle.
        _grip_stop_sent = false;
        _grip_fired     = false;
        _push_active    = false;
        // Seed the vision-lost watchdog so we don't immediately fire on a
        // single frame of perception noise during state entry.
        _approach_last_seen_at = std::chrono::steady_clock::now();
    }

    if (new_state == State::CARRY_HOME) {
        const auto now = std::chrono::steady_clock::now();
        // Stuck-recovery sub-state. NORMAL = follow the spline that
        // CARRY_HOME's plan-block above just installed. Anchor the
        // stuck-detection window at the current pose; it advances each
        // tick we move more than _carry_stuck_dist_threshold_mm.
        _carry_phase             = CarryPhase::NORMAL;
        _carry_phase_entered     = now;
        _carry_recovery_attempts = 0;
        _stuck_anchor_pose       = current;
        _stuck_anchor_at         = now;
    }

    if (new_state == State::RELEASE) {
        servo_release();
    }

    if (new_state == State::TURN_AROUND) {
        // Simple in-place rotation to _home_theta. SEARCH's bounded
        // look-left/look-right sweep handles re-orientation, so this just
        // needs to point the chassis roughly back at home_theta before
        // handing off.
        double home_theta_local;
        { std::lock_guard<std::mutex> lk(_m); home_theta_local = _home_theta; }
        const double delta = wrap_pi(home_theta_local - current.theta);
        constexpr double DELTA_DEADBAND_RAD = 0.05;          // ~3°
        if (std::fabs(delta) < DELTA_DEADBAND_RAD) {
            new_state = State::SEARCH;
        }
    }

    // SEARCH entry side-effects last so any preceding block that rewrites
    // new_state to SEARCH (currently TURN_AROUND's delta-deadband and
    // plan-failure paths) still triggers the side-effects. Drop the
    // skip-streak log gate, clear the latched APPROACH target, reset the
    // look-left/look-right/forward state, abort any leftover spline, and
    // re-latch the gripper open.
    if (new_state == State::SEARCH) {
        _search_skip_streak    = false;
        _last_target_valid     = false;
        _ssearch.initialized   = false;
        _controller.abort();
        servo_release();
    }

    {
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lk(_m);
        // CAS guard: if abort() raced in between our `old_state` read and
        // this write (main thread sets SAFE_STOP via abort()), let abort()
        // win.
        if (_state.load() != old_state) {
            LOG("Mission") << "transition " << state_name(old_state) << " -> "
                           << state_name(new_state)
                           << " aborted (state changed concurrently to "
                           << state_name(_state.load()) << ")";
            return;
        }
        _state.store(new_state);
        _state_entered  = now;
    }
    LOG("Mission") << state_name(old_state) << " -> " << state_name(new_state);
}

void StateMachine::_enter_search(const Pos2D& current) {
    double hx_local, hy_local;
    {
        std::lock_guard<std::mutex> lk(_m);
        hx_local = _home_x;
        hy_local = _home_y;
    }

    const double dist_from_home =
        std::hypot(current.x - hx_local, current.y - hy_local);
    const bool far_from_home = dist_from_home > _search_far_threshold_mm;
    const double stage_theta = far_from_home ? M_PI : 0.0;
    const double stage_x = clamp_d(current.x,
                                   _search_stage_x_min_mm,
                                   _search_stage_x_max_mm);
    const double stage_y = hy_local;

    const bool x_ok     = std::fabs(current.x - stage_x) <= _planned_margin_mm;
    const bool y_ok     = std::fabs(current.y - stage_y) <= _planned_margin_mm;
    const bool theta_ok = std::fabs(wrap_pi(current.theta - stage_theta))
                            <= _search_stage_theta_tol;

    if (x_ok && y_ok && theta_ok) {
        // Already on the home y-line with a valid heading — skip staging.
        _transition(State::SEARCH, current);
        return;
    }

    {
        std::lock_guard<std::mutex> lk(_m);
        _goto_x = stage_x;
        _goto_y = stage_y;
        _goto_theta = stage_theta;
        _goto_pending = true;
        _post_goto_state = State::SEARCH;
    }
    LOG("Mission") << "_enter_search: staging to (" << stage_x
                   << ", " << stage_y << ", " << stage_theta
                   << ") far=" << far_from_home
                   << " from (" << current.x << ", " << current.y
                   << ", " << current.theta << ")";
    _transition(State::GOTO, current);
}

double StateMachine::_carry_repulsion_bias(
        const TrackerResult::Snapshot& snap) const {
    double omega_avoid = 0.0;
    for (const auto& obs : snap.obstacles) {
        if (obs.distance_mm > _carry_avoid_window_mm) continue;
        if (std::fabs(obs.bearing_rad) > M_PI / 2.0) continue;
        const double mag =
            _carry_repulsion_gain
            / std::max(obs.distance_mm, 100.0);
        omega_avoid += -std::copysign(mag, obs.bearing_rad);
    }
    return clamp_d(omega_avoid,
                   -_carry_repulsion_omega_max,
                   +_carry_repulsion_omega_max);
}

bool StateMachine::_plan_path_to(const Pos2D& current,
                                 double gx, double gy, double gtheta,
                                 double margin_mm, double v_max_mm_s) {
    return _plan_path_via_offset(current, gx, gy, gtheta,
                                 margin_mm, v_max_mm_s, 0.0);
}

bool StateMachine::_plan_path_via_offset(const Pos2D& current,
                                         double gx, double gy, double gtheta,
                                         double margin_mm, double v_max_mm_s,
                                         double lateral_offset_mm) {
    // Why: a B-cubic through three collinear points (start, midpoint, goal)
    // degenerates to a straight line. Insert tangent-aligned handles along
    // the start and goal headings so the spline leaves/arrives tangent to
    // those headings and traces a visible curve (Lab 4 visualisation).
    constexpr double HANDLE_FRAC   = 0.4;
    constexpr double HANDLE_MAX_MM = 600.0;

    PathPoint p_start;
    p_start.x = current.x;
    p_start.y = current.y;
    p_start.theta = current.theta;

    PathPoint p_goal;
    p_goal.x = gx;
    p_goal.y = gy;
    p_goal.theta = gtheta;
    p_goal.margin = margin_mm;

    const double dx   = gx - current.x;
    const double dy   = gy - current.y;
    const double dist = std::hypot(dx, dy);
    const double d    = std::min(HANDLE_FRAC * dist, HANDLE_MAX_MM);

    PathPoint p_h_start;
    p_h_start.x = current.x + d * std::cos(current.theta);
    p_h_start.y = current.y + d * std::sin(current.theta);

    PathPoint p_h_goal;
    p_h_goal.x = gx - d * std::cos(gtheta);
    p_h_goal.y = gy - d * std::sin(gtheta);

    std::vector<PathPoint> controls;
    controls.reserve(5);
    controls.push_back(p_start);
    controls.push_back(p_h_start);
    if (lateral_offset_mm != 0.0 && dist > 1e-6) {
        // Obstacle-avoidance lateral push inserted between the handles so
        // the start/goal tangent constraints still hold.
        PathPoint p_mid;
        p_mid.x = 0.5 * (p_h_start.x + p_h_goal.x) + lateral_offset_mm * (-dy / dist);
        p_mid.y = 0.5 * (p_h_start.y + p_h_goal.y) + lateral_offset_mm * ( dx / dist);
        controls.push_back(p_mid);
    }
    controls.push_back(p_h_goal);
    controls.push_back(p_goal);

    std::vector<PathPoint> path = bcubic_spline(
        controls,
        30,
        v_max_mm_s,
        margin_mm,
        150.0 * ROBOT_SPEED,
        30.0);
    if (path.empty()) {
        LOGE("Mission") << "bcubic_spline returned empty search path "
                        << "from (" << current.x << ", " << current.y << ") "
                        << "to (" << gx << ", " << gy << ")";
        return false;
    }
    path.back().margin = margin_mm;
    path.back().theta  = gtheta;
    _controller.set_path(path);
    return true;
}

StateMachine::Command StateMachine::step(const Pos2D& pose,
                                         const Pose2DCov& cov,
                                         Dashboard& dash) {
    Command out;
    // Snapshot everything once so the state body sees a consistent view.
    // One mutex acquire per tick on each of _m and the vision lock
    const TrackerResult::Snapshot snap = _vision.get_snapshot();
    std::chrono::steady_clock::time_point entered_at;
    double hx, hy, ht, hskip, arm;
    int    attempts, max_attempts;
    {
        std::lock_guard<std::mutex> lk(_m);
        entered_at   = _state_entered;
        hx  = _home_x; hy = _home_y; ht = _home_theta;
        hskip = _home_skip_radius_mm;
        arm = _arm_length_mm;
        attempts     = _approach_attempts;
        max_attempts = _max_approach_attempts;
    }
    const double t_in    = seconds_since(entered_at);
    const State s = _state.load();

    // ---- Obstacle world-frame memory ----
    // Project each this-frame obstacle observation to world; merge into
    // the nearest existing entry of the same color within
    // _obstacle_merge_radius_mm (EMA blend) or append a fresh entry.
    // Then drop entries that haven't been refreshed for
    // _obstacle_forget_s.
    {
        const auto now = std::chrono::steady_clock::now();
        for (const auto& obs : snap.obstacles) {
            if (obs.distance_mm <= 0.0) continue;
            double ox, oy;
            polar_to_world(pose, obs.bearing_rad, obs.distance_mm, ox, oy);
            int    best = -1;
            double best_d = _obstacle_merge_radius_mm;
            for (size_t i = 0; i < _obstacles.entries.size(); ++i) {
                const auto& e = _obstacles.entries[i];
                if (e.color_id != obs.color_id) continue;
                const double d = std::hypot(ox - e.x_mm, oy - e.y_mm);
                if (d < best_d) { best_d = d; best = static_cast<int>(i); }
            }
            if (best >= 0) {
                // EMA blend so spurious jitter doesn't snap the position.
                auto& e = _obstacles.entries[best];
                constexpr double ALPHA = 0.35;
                e.x_mm = (1.0 - ALPHA) * e.x_mm + ALPHA * ox;
                e.y_mm = (1.0 - ALPHA) * e.y_mm + ALPHA * oy;
                e.last_seen = now;
            } else {
                ObstacleMemory::Entry e;
                e.x_mm = ox; e.y_mm = oy;
                e.color_id = obs.color_id;
                e.last_seen = now;
                _obstacles.entries.push_back(e);
            }
        }
        // Decay
        _obstacles.entries.erase(
            std::remove_if(_obstacles.entries.begin(), _obstacles.entries.end(),
                [&](const ObstacleMemory::Entry& e) {
                    return seconds_since(e.last_seen) > _obstacle_forget_s;
                }),
            _obstacles.entries.end());
    }

    // Box position derived from robot pose + arm length along heading.
    double box_x, box_y;
    polar_to_world(pose, 0.0, arm, box_x, box_y);
    const double box_to_home = std::hypot(box_x - hx, box_y - hy);

    switch (s) {
        case State::IDLE:
            out.mode = Command::Mode::STOP;
            break;

        case State::SEARCH: {
            // Keep the gripper latched OPEN for the entire SEARCH state, not
            // just on entry. Idempotent: servo_tick() no-ops once cur == target.
            servo_release();

            // Highest priority: if we somehow already hold a box (e.g. we
            // re-entered SEARCH mid-mission), do NOT spin — it would fling
            // the cargo. Go straight to deliver.
            if (snap.captured) {
                _transition(State::CARRY_HOME, pose);
                out.mode = Command::Mode::PATH_FOLLOW;
                break;
            }

            const bool target_visible = (!snap.lost && snap.has_target);
            bool at_home_skip = false;

            // Vision has a target -> engage, but enforce the retry budget.
            if (target_visible) {
                // At-home filter: if the tracker has a usable distance and
                // the detected box already sits inside the home zone, this
                // is a previously-delivered box. Skip it (stay in SEARCH,
                // keep searching) so we don't waste an attempt re-picking
                // something that's already where it belongs.
                if (snap.distance_mm > 0.0) {
                    double det_x, det_y;
                    polar_to_world(pose, snap.bearing_rad, snap.distance_mm,
                                   det_x, det_y);
                    const double det_to_home = std::hypot(det_x - hx,
                                                          det_y - hy);
                    // Half-plane skip: project (det - home) onto the
                    // home-forward axis (cos(ht), sin(ht)). Negative
                    // projection = box sits behind home along the
                    // search-forward axis, which is wall territory we
                    // never sweep into. Combined with the radius skip
                    // below for off-center drops on the arena side.
                    const double along_home =
                        (det_x - hx) * std::cos(ht) +
                        (det_y - hy) * std::sin(ht);
                    const bool behind_home = along_home < 0.0;
                    // Obstacle-block: skip target if it sits inside the
                    // exclusion radius of any remembered obstacle world
                    // position. Catches the chain-push case (blocker
                    // adjacent to target) before APPROACH even fires.
                    bool blocked_by_obs = false;
                    for (const auto& e : _obstacles.entries) {
                        const double d = std::hypot(det_x - e.x_mm,
                                                    det_y - e.y_mm);
                        if (d < _obstacle_block_radius_mm) {
                            blocked_by_obs = true;
                            break;
                        }
                    }
                    if (det_to_home < hskip || behind_home || blocked_by_obs) {
                        if (!_search_skip_streak) {
                            const char* reason =
                                blocked_by_obs ? "obs_block"
                                : behind_home  ? "behind_home"
                                               : "at_home";
                            LOG("Mission")
                                << "Skipping box at world ("
                                << det_x << ", " << det_y << "), "
                                << det_to_home << " mm from home, "
                                << "along_home=" << along_home
                                << " mm, reason=" << reason;
                            _search_skip_streak = true;
                        }
                        at_home_skip = true;
                    }
                    // Failed-target skip: same idea as at-home, but for a
                    // box we just couldn't engage. The skip is bounded in
                    // both space and time so a *different* box at a similar
                    // bearing later isn't ignored.
                    if (!at_home_skip && _last_failed_target_valid) {
                        const double age =
                            seconds_since(_last_failed_at);
                        const double d_to_fail = std::hypot(
                            det_x - _last_failed_target_x,
                            det_y - _last_failed_target_y);
                        if (age <= _failed_target_skip_s &&
                            d_to_fail <= _failed_target_skip_radius_mm) {
                            at_home_skip = true;
                        } else if (age > _failed_target_skip_s) {
                            _last_failed_target_valid = false;
                        }
                    }
                }

                if (!at_home_skip) {
                    // Box is engageable — reset the skip streak before we
                    // hand off to APPROACH.
                    _search_skip_streak = false;

                    // Budget-exhausted SAFE_STOP removed by request:
                    // SEARCH runs indefinitely until the user sends
                    // SIGINT (Ctrl+C). `attempts` keeps incrementing for
                    // diagnostics but is no longer fatal. Log only on
                    // the crossing tick (== max), not every subsequent
                    // APPROACH transition.
                    if (attempts == max_attempts) {
                        LOG("Mission") << "Approach budget reached ("
                                       << attempts << "/" << max_attempts
                                       << "); continuing anyway.";
                    }
                    // NOTE: do NOT increment _approach_attempts here. The
                    // attempt counter only ticks on a *real* failed try (i.e.
                    // an APPROACH that ran out the full _approach_timeout_s
                    // without capturing). Bumping on entry meant a vision
                    // flicker — APPROACH→vision_lost→SEARCH→APPROACH — burned
                    // a full attempt each cycle, and a flaky HSV mask could
                    // bankrupt the budget in 3 flickers without the robot ever
                    // really engaging the box. See the APPROACH-timeout case
                    // below for where the increment now lives.
                    _transition(State::APPROACH, pose);
                    out.mode = Command::Mode::DIRECT;
                    out.v_mm_s = _approach_v;
                    out.omega_rad_s = clamp_d(_approach_k_omega * snap.bearing_rad,
                                              -_approach_omega_max,
                                              _approach_omega_max);
                    break;
                }
            }

            // SEARCH timeout SAFE_STOP removed by request: SEARCH runs
            // indefinitely until the user sends SIGINT (Ctrl+C).
            // _search_timeout_s is retained for future diagnostics.

            if (!target_visible) {
                _search_skip_streak = false;
            }

            // Fixed-axis look-left / look-right / forward.
            // forward_heading_ref is snapshotted once on first-tick init
            // and held constant for this whole SEARCH entry. Every cycle's
            // sweep targets are absolute (ref ± π/2) and the forward leg
            // realigns to ref before driving. No per-cycle drift.
            if (!_ssearch.initialized) {
                _ssearch.phase = SimpleSearch::Phase::LOOK_LEFT;
                _ssearch.forward_traveled_mm   = 0.0;
                _ssearch.obstacle_rotated_rad  = 0.0;
                _ssearch.obstacle_dir_sign     = 0.0;
                _ssearch.look_right_rotated_rad = 0.0;
                _ssearch.forward_heading_ref   = pose.theta;
                _ssearch.in_realign            = false;
                _ssearch.realign_stable_count  = 0;
                _ssearch.last_x     = pose.x;
                _ssearch.last_y     = pose.y;
                _ssearch.last_theta = pose.theta;
                _ssearch.initialized = true;
            }

            const double ref = _ssearch.forward_heading_ref;

            if (_ssearch.phase == SimpleSearch::Phase::LOOK_LEFT) {
                // Target: pose.theta == ref + π/2. While CCW-rotating
                // toward that target, wrap_pi(target - pose) is positive
                // and decreases to 0 (or just-negative on overshoot).
                const double err =
                    wrap_pi((ref + M_PI / 2.0) - pose.theta);
                _ssearch.last_theta = pose.theta;
                if (err <= 0.0) {
                    _ssearch.phase = SimpleSearch::Phase::LOOK_RIGHT;
                    _ssearch.look_right_rotated_rad = 0.0;
                    _ssearch.last_theta = pose.theta;
                }
                out.mode = Command::Mode::DIRECT;
                out.v_mm_s = 0.0;
                out.omega_rad_s = _search_omega;          // CCW
                break;
            }

            if (_ssearch.phase == SimpleSearch::Phase::LOOK_RIGHT) {
                // Spans ≈ π through the back of the robot — can't use an
                // absolute target check (wrap_pi(-π) = +π would satisfy
                // err ≥ 0 on tick 0). Integrate signed Δθ from phase
                // entry and exit once we've rotated CW by ≥ π.
                const double dth = wrap_pi(pose.theta - _ssearch.last_theta);
                _ssearch.look_right_rotated_rad += dth;
                _ssearch.last_theta = pose.theta;
                if (_ssearch.look_right_rotated_rad <= -M_PI) {
                    _ssearch.phase = SimpleSearch::Phase::FORWARD;
                    _ssearch.forward_traveled_mm   = 0.0;
                    _ssearch.obstacle_rotated_rad  = 0.0;
                    _ssearch.obstacle_dir_sign     = 0.0;
                    // Need to realign from -π/2 back to ref before
                    // driving forward.
                    _ssearch.in_realign            = true;
                    _ssearch.realign_stable_count  = 0;
                    _ssearch.last_x = pose.x;
                    _ssearch.last_y = pose.y;
                }
                out.mode = Command::Mode::DIRECT;
                out.v_mm_s = 0.0;
                out.omega_rad_s = -_search_omega;         // CW
                break;
            }

            // Phase::FORWARD
            const double dth_signed =
                wrap_pi(pose.theta - _ssearch.last_theta);

            if (app::obstacle_ahead.load()) {
                // Pick a rotation direction once per obstacle from lidar
                // side clearance — turn toward the freer half-arc.
                if (_ssearch.obstacle_dir_sign == 0.0) {
                    const double L = app::obstacle_left_mm.load();
                    const double R = app::obstacle_right_mm.load();
                    _ssearch.obstacle_dir_sign = (L >= R) ? +1.0 : -1.0;
                }
                _ssearch.obstacle_rotated_rad += dth_signed;
                _ssearch.last_theta = pose.theta;
                if (std::fabs(_ssearch.obstacle_rotated_rad)
                        >= _search_obstacle_max_rotation_rad) {
                    // Couldn't clear obstacle in a half-turn. Abandon the
                    // leg and restart the sweep from the new heading; if
                    // the situation is truly stuck the mission-wide
                    // _search_timeout_s will kill SEARCH.
                    _ssearch.phase = SimpleSearch::Phase::LOOK_LEFT;
                    _ssearch.obstacle_rotated_rad = 0.0;
                    _ssearch.obstacle_dir_sign    = 0.0;
                    _ssearch.in_realign           = false;
                    _ssearch.realign_stable_count = 0;
                    out.mode = Command::Mode::DIRECT;
                    out.v_mm_s = 0.0;
                    out.omega_rad_s = 0.0;
                    break;
                }
                out.mode = Command::Mode::DIRECT;
                out.v_mm_s = 0.0;
                out.omega_rad_s = _ssearch.obstacle_dir_sign * _search_omega;
                // Obstacle dodge implies we'll need to realign-to-ref
                // when it clears.
                _ssearch.in_realign           = true;
                _ssearch.realign_stable_count = 0;
                break;
            }

            // Hysteresis realign to forward_heading_ref. Enter when
            // |err| > enter_rad (or in_realign already latched); exit
            // only after |err| < exit_rad for stable_ticks consecutive
            // ticks. Kills the cox-jitter flap near the deadband.
            const double heading_err = wrap_pi(ref - pose.theta);
            if (!_ssearch.in_realign
                && std::fabs(heading_err) > _search_realign_enter_rad) {
                _ssearch.in_realign = true;
                _ssearch.realign_stable_count = 0;
            }
            if (_ssearch.in_realign) {
                _ssearch.last_theta = pose.theta;
                if (std::fabs(heading_err) < _search_realign_exit_rad) {
                    if (++_ssearch.realign_stable_count
                            >= _search_realign_stable_ticks) {
                        _ssearch.in_realign           = false;
                        _ssearch.realign_stable_count = 0;
                        _ssearch.obstacle_rotated_rad = 0.0;
                        _ssearch.obstacle_dir_sign    = 0.0;
                        // Stable; fall through to forward drive on the
                        // next tick.
                        out.mode = Command::Mode::DIRECT;
                        out.v_mm_s = 0.0;
                        out.omega_rad_s = 0.0;
                        break;
                    }
                } else {
                    _ssearch.realign_stable_count = 0;
                }
                out.mode = Command::Mode::DIRECT;
                out.v_mm_s = 0.0;
                out.omega_rad_s = (heading_err >= 0.0 ? +1.0 : -1.0)
                                  * _search_omega;
                break;
            }
            {
                // Signed forward odometer: project the body-frame step
                // onto the heading so backward push and lateral jitter do
                // not inflate progress.
                const double dx = pose.x - _ssearch.last_x;
                const double dy = pose.y - _ssearch.last_y;
                const double step = dx * std::cos(pose.theta)
                                  + dy * std::sin(pose.theta);
                if (step > 0.0) _ssearch.forward_traveled_mm += step;
                _ssearch.last_x     = pose.x;
                _ssearch.last_y     = pose.y;
                _ssearch.last_theta = pose.theta;
            }
            // X-far turn-back: if pose.x crosses _search_x_far_turn_mm
            // while the heading reference still points into the arena
            // (cos(ref) > 0), flip the reference 180° and restart the
            // sweep cycle from LOOK_LEFT. The cos() guard makes this a
            // one-shot per SEARCH entry — once facing home, we don't
            // re-flip on the way back.
            if (pose.x >= _search_x_far_turn_mm
                && std::cos(_ssearch.forward_heading_ref) > 0.0) {
                LOG("Mission")
                    << "SEARCH: x=" << pose.x
                    << " mm >= turn-back " << _search_x_far_turn_mm
                    << " mm; flipping heading ref toward home.";
                _ssearch.forward_heading_ref =
                    wrap_pi(_ssearch.forward_heading_ref + M_PI);
                // Skip LOOK_LEFT — its err would be ~-π/2 against the
                // flipped ref and it'd transition out on the next tick
                // anyway (after one tick of wrong-direction CCW). Jump
                // straight to LOOK_RIGHT, which performs the U-turn CW.
                _ssearch.phase                  = SimpleSearch::Phase::LOOK_RIGHT;
                _ssearch.forward_traveled_mm    = 0.0;
                _ssearch.look_right_rotated_rad = 0.0;
                _ssearch.obstacle_rotated_rad   = 0.0;
                _ssearch.obstacle_dir_sign      = 0.0;
                _ssearch.in_realign             = true;
                _ssearch.realign_stable_count   = 0;
                _ssearch.last_theta             = pose.theta;
                out.mode = Command::Mode::DIRECT;
                out.v_mm_s = 0.0;
                out.omega_rad_s = 0.0;
                break;
            }
            if (_ssearch.forward_traveled_mm >= _search_forward_target_mm) {
                _ssearch.phase = SimpleSearch::Phase::LOOK_LEFT;
                _ssearch.last_theta = pose.theta;
                out.mode = Command::Mode::DIRECT;
                out.v_mm_s = 0.0;
                out.omega_rad_s = 0.0;
                break;
            }
            out.mode = Command::Mode::DIRECT;
            out.v_mm_s = _search_v_forward_mm_s;
            out.omega_rad_s = 0.0;
            break;
        }

        case State::APPROACH: {
            // Reset the "deceleration tick sent" latch whenever we are not
            // about to grip (no captured/grabbable signal AND grip hasn't
            // fired yet). A brief vision flicker into snap.captured or
            // snap.grabbable would otherwise leave _grip_stop_sent=true and
            // skip the deceleration tick on the next genuine grip trigger
            // — closing the jaws at full chase speed. Push-phase latch
            // resets on the same gate so a single-frame grabbable blink
            // doesn't leave the pushed-distance accumulator stale.
            // Wall directly ahead (lidar fires) and no vision-detected
            // obstacle box in front → it's a wall, not something we want
            // to swerve around. Boxes sit below the lidar plane so any
            // lidar return at <230 mm in the ±60° forward arc that isn't
            // a known vision blob must be a wall.
            const bool wall_close = app::obstacle_ahead.load()
                                 && snap.obstacles.empty();
            // Hold the push latch through a brief grabbable flicker only
            // when wall_close is true — wall contact can squish the
            // target out of the camera's view, but we still want the
            // grip to fire from the early-capture branch below.
            if (!_grip_fired && !snap.captured && !snap.grabbable
                && !(_push_active && wall_close)) {
                _grip_stop_sent = false;
                _push_active    = false;
            }

            // 1. Servo grab already fired. Treat the grip as committed: once
            //    the jaws are closing, the gripper arms occlude the box and
            //    `snap.captured` becomes unreliable, so we don't wait on
            //    vision. Hold v=0 until servo_done(0) reports the jaws have
            //    finished slewing CLOSED (full 1.6s at 1°/20ms), then
            //    transition to CARRY_HOME. A 3s watchdog prevents a wedged
            //    servo from deadlocking APPROACH forever.
            if (_grip_fired) {
                const bool slew_done = servo_done(0);
                const bool watchdog  = seconds_since(_grip_fired_at) >= 3.0;
                if (slew_done || watchdog) {
                    if (watchdog && !slew_done) {
                        LOGE("Mission") << "Grip watchdog fired after 3s "
                                           "without servo_done; proceeding to "
                                           "CARRY_HOME anyway.";
                    }
                    { std::lock_guard<std::mutex> lk(_m); _approach_attempts = 0; }
                    _transition(State::CARRY_HOME, pose);
                    out.mode = Command::Mode::PATH_FOLLOW;
                    break;
                }
                out.mode = Command::Mode::DIRECT;
                out.v_mm_s = 0.0;
                out.omega_rad_s = 0.0;
                break;
            }

            // 2. Pre-grip captured (rare: e.g. the box drifts inside the
            //    100mm capture radius before the bottom-fill grab gate
            //    fires). Previously this short-circuited straight to
            //    CARRY_HOME without ever firing servo_grab(), leaving the
            //    gripper OPEN — robot carried nothing home. Run the same
            //    two-tick grip sequence as the grabbable branch so the
            //    next tick falls into the _grip_fired path and waits for
            //    servo_done(0) before driving.
            if (snap.captured) {
                if (!_grip_stop_sent) {
                    _grip_stop_sent = true;
                    out.mode = Command::Mode::DIRECT;
                    out.v_mm_s = 0.0;
                    out.omega_rad_s = 0.0;
                    break;
                }
                servo_grab();
                _grip_fired    = true;
                _grip_fired_at = std::chrono::steady_clock::now();
                out.mode = Command::Mode::DIRECT;
                out.v_mm_s = 0.0;
                out.omega_rad_s = 0.0;
                break;
            }

            // 3. Box is in the grab zone. Sequence is:
            //    - first grabbable tick: latch push phase; drive slowly
            //      forward to seat the target between the gripper arms.
            //    - push complete (distance or timeout): latch
            //      _grip_stop_sent + command v=0 for one tick to let the
            //      motor decelerate before the servo latches.
            //    - next tick: call servo_grab() (non-blocking: latches the
            //      target angle; servo_tick() in loop_control walks the
            //      PWM in STEP_DEG / STEP_MS chunks). _grip_fired stays
            //      true and v=0 is held by branch (1) above while the
            //      slew completes.
            if (snap.grabbable) {
                if (!_push_active) {
                    _push_active = true;
                    _push_started_at = std::chrono::steady_clock::now();
                    _push_start_x = pose.x;
                    _push_start_y = pose.y;
                    LOG("Mission") << "APPROACH: push-then-grip start";
                }
                const double dx = pose.x - _push_start_x;
                const double dy = pose.y - _push_start_y;
                const double pushed = std::hypot(dx, dy);
                const double t_pushing = seconds_since(_push_started_at);
                // wall_close (computed at the top of this case) short-
                // circuits the push: target is already pinned against a
                // wall, so seating is established and further push only
                // crushes the gripper into the wall.
                const bool push_done = (pushed >= _approach_push_distance_mm)
                                    || (t_pushing >= _approach_push_timeout_s)
                                    || wall_close;
                if (!push_done) {
                    out.mode = Command::Mode::DIRECT;
                    out.v_mm_s = _approach_push_v;
                    out.omega_rad_s = 0.0;
                    break;
                }
                if (!_grip_stop_sent) {
                    _grip_stop_sent = true;
                    out.mode = Command::Mode::DIRECT;
                    out.v_mm_s = 0.0;
                    out.omega_rad_s = 0.0;
                    break;
                }
                servo_grab();
                _grip_fired    = true;
                _grip_fired_at = std::chrono::steady_clock::now();
                out.mode = Command::Mode::DIRECT;
                out.v_mm_s = 0.0;
                out.omega_rad_s = 0.0;
                break;
            }

            // 4. Chase phase: latch the target's world position whenever
            //    vision has a clean reading, then visual-servo toward it.
            const bool target_visible = (!snap.lost && snap.has_target);
            if (target_visible && snap.distance_mm > 0.0) {
                polar_to_world(pose, snap.bearing_rad, snap.distance_mm,
                               _last_target_x, _last_target_y);
                _last_target_valid = true;
            }
            if (target_visible) {
                _approach_last_seen_at = std::chrono::steady_clock::now();
            }

            if (!target_visible) {
                // Watchdog: if vision has been continuously lost for
                // _approach_vision_lost_timeout_s, give up and fall back
                // to SEARCH instead of dead-reckoning toward a stale
                // latched target. No budget charge — this is a perception
                // gap, same class as the no-latched-target branch below.
                const double t_lost = seconds_since(_approach_last_seen_at);
                if (t_lost > _approach_vision_lost_timeout_s) {
                    LOG("Mission") << "APPROACH: vision lost for "
                                   << t_lost << " s; -> SEARCH";
                    // Direct SEARCH (no staging): vision-lost loops can
                    // burn through _approach_attempts; the GOTO halt on
                    // every cycle wastes ~1-2 s with no benefit since
                    // we're not actually mid-arena far from the search
                    // axis here.
                    _transition(State::SEARCH, pose);
                    out.mode = Command::Mode::STOP;
                    break;
                }
                // Vision dropped briefly. If we still have a latched world
                // target and we haven't reached it yet, keep driving toward
                // it instead of spinning — re-entering SEARCH here was the
                // root cause of the "lose target → rotate → lose new
                // bearing" thrash. No budget charge: this is a perception
                // gap, not a failed engagement.
                if (_last_target_valid) {
                    const double dx = _last_target_x - pose.x;
                    const double dy = _last_target_y - pose.y;
                    const double dist = std::hypot(dx, dy);
                    if (dist > _dead_reckon_arrival_mm) {
                        const double bearing_err =
                            wrap_pi(std::atan2(dy, dx) - pose.theta);
                        out.mode = Command::Mode::DIRECT;
                        out.v_mm_s = _approach_v;
                        out.omega_rad_s =
                            clamp_d(_approach_k_omega * bearing_err,
                                    -_approach_omega_max,
                                    _approach_omega_max);
                        break;
                    }
                }
                _transition(State::SEARCH, pose);
                out.mode = Command::Mode::STOP;
                break;
            }

            if (t_in > _approach_timeout_s) {
                // Box visible but we cannot engage it within the timeout.
                // This is a real failed attempt charge the budget here, and
                // remember where the failed box sat so SEARCH skips it for
                // a while instead of immediately re-engaging.
                { std::lock_guard<std::mutex> lk(_m); ++_approach_attempts; }
                if (_last_target_valid) {
                    _last_failed_target_valid = true;
                    _last_failed_target_x = _last_target_x;
                    _last_failed_target_y = _last_target_y;
                    _last_failed_at = std::chrono::steady_clock::now();
                }
                _transition(State::SEARCH, pose);
                out.mode = Command::Mode::STOP;
                break;
            }

            // Visual servo: forward at constant speed, steer with bearing
            // (P controller). Bearing is +left (tracker convention);
            // motor omega is CCW-positive, which matches: +bearing -> +omega.
            // Add a reactive repulsion bias from each obstacle that sits
            // in front of the target on the visual axis — steers around
            // the blocker while still chasing the target.
            double omega_target = _approach_k_omega * snap.bearing_rad;
            double omega_avoid  = 0.0;
            for (const auto& obs : snap.obstacles) {
                if (obs.distance_mm >= snap.distance_mm) continue;
                if (std::fabs(obs.bearing_rad) > M_PI / 2.0) continue;
                const double mag =
                    _approach_repulsion_gain
                    / std::max(obs.distance_mm, 100.0);
                // Steer opposite the obstacle bearing: +bearing (left) =>
                // negative omega bias (steer right).
                omega_avoid += -std::copysign(mag, obs.bearing_rad);
            }
            out.mode = Command::Mode::DIRECT;
            out.v_mm_s = _approach_v;
            out.omega_rad_s = clamp_d(omega_target + omega_avoid,
                                      -_approach_omega_max,
                                      _approach_omega_max);
            break;
        }

        case State::CARRY_HOME: {
            const double dist_to_home =
                std::hypot(pose.x - hx, pose.y - hy);
            const bool at_home = dist_to_home <= _home_margin;

            // Entry-plan retry. If _transition(CARRY_HOME) couldn't
            // build a spline, _controller has no path. Retry from the
            // live pose each tick until it succeeds. Only relevant in
            // NORMAL phase — REVERSING uses direct backward commands
            // (no path) and CURLING installs its path inside the
            // reverse-done branch below.
            if (_carry_phase == CarryPhase::NORMAL
                && _controller.total() == 0) {
                const double tan_to_goal =
                    std::atan2(hy - pose.y, hx - pose.x);
                if (!_plan_path_to(pose, hx, hy, tan_to_goal,
                                   _home_margin, _planned_v_max_mm_s)) {
                    out.mode = Command::Mode::STOP;
                    break;
                }
                LOG("Mission")
                    << "CARRY_HOME: deferred plan succeeded from step() "
                       "retry";
                _stuck_anchor_pose = pose;
                _stuck_anchor_at   = std::chrono::steady_clock::now();
            }

            if (_controller.has_arrived()) {
                if (!at_home) {
                    // PurePursuit reached the end of its spline but
                    // Kalman says we're still > _home_margin from home.
                    // Build a fresh spline and keep dragging instead of
                    // stranding the box mid-arena.
                    const double tan_to_goal =
                        std::atan2(hy - pose.y, hx - pose.x);
                    LOG("Mission")
                        << "CARRY_HOME: pure-pursuit arrived but Kalman is "
                        << dist_to_home << " mm from home (> " << _home_margin
                        << " mm); re-planning to home.";
                    if (!_plan_path_to(pose, hx, hy, tan_to_goal,
                                       _home_margin, _planned_v_max_mm_s)) {
                        // No path; the no-path branch above will retry
                        // next tick.
                        out.mode = Command::Mode::STOP;
                        break;
                    }
                    _stuck_anchor_pose = pose;
                    _stuck_anchor_at   = std::chrono::steady_clock::now();
                    out.mode = Command::Mode::PATH_FOLLOW;
                    break;
                }
                _transition(State::RELEASE, pose);
                out.mode = Command::Mode::DIRECT;
                out.v_mm_s = _release_v;
                out.omega_rad_s = 0.0;
                break;
            }
            if (t_in > _carry_timeout_s) {
                _transition(State::SAFE_STOP, pose);
                out.mode = Command::Mode::STOP;
                break;
            }

            // Stuck-recovery sub-state machine. Either drives the carry
            // spline (NORMAL / CURLING) and watches for lack of progress,
            // or executes a straight reverse leg (REVERSING). The phase
            // entered on CARRY_HOME entry is NORMAL; CURLING is entered
            // after a successful reverse with a freshly planned offset
            // spline.
            const auto now = std::chrono::steady_clock::now();
            if (_carry_phase == CarryPhase::REVERSING) {
                const double reversed =
                    std::hypot(pose.x - _reverse_start_pose.x,
                               pose.y - _reverse_start_pose.y);
                const double t_in_phase =
                    seconds_since(_carry_phase_entered);
                const bool done = reversed >= _carry_recovery_reverse_mm;
                const bool timed_out =
                    t_in_phase > _carry_recovery_reverse_timeout_s;
                if (done || timed_out) {
                    // Pick a side. attempts is 0 before the first curl,
                    // so the first curl goes left, the second right.
                    const bool curl_left = (_carry_recovery_attempts == 0);
                    const double sign = curl_left ? +1.0 : -1.0;
                    const double tan_to_goal =
                        std::atan2(hy - pose.y,
                                   hx - pose.x);
                    if (!_plan_path_via_offset(
                            pose, hx, hy, tan_to_goal,
                            _home_margin, _planned_v_max_mm_s,
                            sign * _carry_recovery_curl_offset_mm)) {
                        // Stay stationary in REVERSING phase; the
                        // reverse-done check is time/distance-based so
                        // it'll re-evaluate the plan next tick. Don't
                        // bump _carry_recovery_attempts until a plan
                        // actually loads.
                        out.mode = Command::Mode::DIRECT;
                        out.v_mm_s = 0.0;
                        out.omega_rad_s = 0.0;
                        break;
                    }
                    LOG("Mission")
                        << "CARRY_HOME: recovery — reversed " << reversed
                        << " mm" << (timed_out ? " (timed out)" : "")
                        << "; curling "
                        << (curl_left ? "LEFT" : "RIGHT")
                        << " (attempt "
                        << (_carry_recovery_attempts + 1) << "/"
                        << _carry_recovery_max_attempts << ")";
                    _carry_phase             = CarryPhase::CURLING;
                    _carry_phase_entered     = now;
                    _stuck_anchor_pose       = pose;
                    _stuck_anchor_at         = now;
                    _carry_recovery_attempts++;
                    out.omega_bias_rad_s = _carry_repulsion_bias(snap);
                    out.mode = Command::Mode::PATH_FOLLOW;
                    break;
                }
                // Still reversing — direct backward command.
                out.mode = Command::Mode::DIRECT;
                out.v_mm_s = -_carry_recovery_reverse_v;
                out.omega_rad_s = 0.0;
                break;
            }

            // NORMAL or CURLING: drive the spline and watch for stuck.
            const double moved =
                std::hypot(pose.x - _stuck_anchor_pose.x,
                           pose.y - _stuck_anchor_pose.y);
            if (moved > _carry_stuck_dist_threshold_mm) {
                // Sliding window: motion exceeded threshold, advance the
                // anchor. As long as the robot keeps inching forward,
                // anchor_at stays fresh and stuck never fires.
                _stuck_anchor_pose = pose;
                _stuck_anchor_at   = now;
            } else if (seconds_since(_stuck_anchor_at)
                       > _carry_stuck_window_s) {
                if (_carry_recovery_attempts
                    >= _carry_recovery_max_attempts) {
                    LOG("Mission")
                        << "CARRY_HOME: stuck again after "
                        << _carry_recovery_attempts
                        << " recovery attempts; releasing in place ("
                        << dist_to_home
                        << " mm from home) and returning to SEARCH.";
                    _transition(State::RELEASE, pose);
                    out.mode = Command::Mode::DIRECT;
                    out.v_mm_s = _release_v;
                    out.omega_rad_s = 0.0;
                    break;
                }
                LOG("Mission")
                    << "CARRY_HOME: stuck — "
                    << moved << " mm in last "
                    << seconds_since(_stuck_anchor_at)
                    << " s; reversing "
                    << _carry_recovery_reverse_mm << " mm.";
                _controller.abort();
                _carry_phase         = CarryPhase::REVERSING;
                _carry_phase_entered = now;
                _reverse_start_pose  = pose;
                out.mode = Command::Mode::DIRECT;
                out.v_mm_s = -_carry_recovery_reverse_v;
                out.omega_rad_s = 0.0;
                break;
            }
            out.omega_bias_rad_s = _carry_repulsion_bias(snap);
            out.mode = Command::Mode::PATH_FOLLOW;
            break;
        }

        case State::RELEASE: {
            // Hard timeout: hand off to TURN_AROUND even if the box is still
            // tagged captured (probably the tracker holding the latch on a
            // static pink blob it can still see between/behind the arms).
            if (t_in >= _release_hard_timeout_s) {
                _transition(State::TURN_AROUND, pose);
                out.mode = Command::Mode::STOP;
                break;
            }
            // Normal exit: minimum back-up time elapsed AND the tracker
            // confirms the box is no longer in the retention ROI.
            if (t_in >= _release_seconds && !snap.captured) {
                _transition(State::TURN_AROUND, pose);
                out.mode = Command::Mode::STOP;
                break;
            }
            // Keep backing up.
            out.mode = Command::Mode::DIRECT;
            out.v_mm_s = _release_v;
            out.omega_rad_s = 0.0;
            break;
        }

        case State::TURN_AROUND: {
            // Gentle in-place rotation to _home_theta. P-controller with
            // 2.0 rad/s/rad gain capped at 1.0 rad/s — slow enough that
            // cox gets several scan corrections during the spin and
            // wheel-slip drift stays bounded.
            double home_theta_local;
            { std::lock_guard<std::mutex> lk(_m); home_theta_local = _home_theta; }
            const double delta = wrap_pi(home_theta_local - pose.theta);

            if (std::fabs(delta) < 0.05) {  // ~3°
                _enter_search(pose);
                out.mode = Command::Mode::STOP;
                break;
            }
            if (t_in > _turn_around_timeout_s) {
                LOG("Mission") << "REALIGN timed out after " << t_in
                               << " s; bailing to SEARCH.";
                _enter_search(pose);
                out.mode = Command::Mode::STOP;
                break;
            }
            out.mode = Command::Mode::DIRECT;
            out.v_mm_s = 0.0;
            out.omega_rad_s = clamp_d(2.0 * delta, -1.0, 1.0);
            break;
        }

        case State::GOTO: {
            // First tick after go_to(): build the spline from the live pose
            // to the latched target. (go_to() can't do this itself because
            // the pose lives on the control thread.)
            bool pending;
            double tx, ty, tth, vmax, margin;
            {
                std::lock_guard<std::mutex> lk(_m);
                pending = _goto_pending;
                tx = _goto_x; ty = _goto_y; tth = _goto_theta;
                vmax = _planned_v_max_mm_s; margin = _planned_margin_mm;
                if (pending) _goto_pending = false;
            }
            if (pending) {
                if (!_plan_path_to(pose, tx, ty, tth, margin, vmax)) {
                    _transition(State::SAFE_STOP, pose);
                    out.mode = Command::Mode::STOP;
                    break;
                }
                LOG("Mission") << "GOTO: spline loaded, "
                               << _controller.total() << " waypoints";
            }

            // Rising-edge into halt-and-localize before declaring DONE so
            // the final pose used by goto_arrived() reflects a settled KF
            // rather than the moment PurePursuit's geometric arrival fired.
            if (!_halting && _controller.has_arrived()) {
                _halting       = true;
                _halt_started  = std::chrono::steady_clock::now();
                _halt_target_x = tx;
                _halt_target_y = ty;
                out.mode = Command::Mode::STOP;
                break;
            }

            if (_halting) {
                const auto elapsed_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - _halt_started).count();
                const double sigma_xy = std::sqrt(std::max(cov.xx, cov.yy));
                const bool min_done = elapsed_ms >= HALT_MIN_MS;
                const bool settled  = sigma_xy <= HALT_SIGMA_XY_TARGET_MM;
                const bool timed_out = elapsed_ms >= HALT_MAX_MS;
                if ((min_done && settled) || timed_out) {
                    const double mahal2 = mahalanobis2_xy(
                        pose, cov, _halt_target_x, _halt_target_y);
                    const bool gate_pass = (mahal2 <= MAHAL_GATE_CHI2);
                    LOG("Mission") << "GOTO halt done: " << elapsed_ms
                                   << "ms sigma_xy=" << sigma_xy
                                   << "mm mahal2=" << mahal2
                                   << (gate_pass ? " (pass)"
                                                 : " (FAIL — accepting)");
                    _halting = false;
                    State next;
                    {
                        std::lock_guard<std::mutex> lk(_m);
                        next = _post_goto_state;
                        _post_goto_state = State::DONE;
                    }
                    // Chain target consumed: transition directly. Re-running
                    // _enter_search here would re-evaluate stage_theta from
                    // the post-halt pose and could oscillate when the
                    // staged x sits near _search_far_threshold_mm.
                    _transition(next, pose);
                }
                out.mode = Command::Mode::STOP;
                break;
            }
            out.mode = Command::Mode::PATH_FOLLOW;
            break;
        }

        case State::TEST: {
            // Plan-on-entry / plan-on-arrival: one-shot per leg. Set by
            // start_test() and by the arrival branch below.
            if (_test_replan_needed) {
                if (_test_step >= TEST_WAYPOINT_COUNT) {
                    LOG("Mission") << "TEST: all legs complete";
                    _transition(State::DONE, pose);
                    out.mode = Command::Mode::STOP;
                    break;
                }
                const TestWaypoint& wp = TEST_WAYPOINTS[_test_step];
                if (!_plan_path_to(pose, wp.x, wp.y, wp.theta,
                                   _planned_margin_mm, _planned_v_max_mm_s)) {
                    LOGE("Mission") << "TEST: leg " << (_test_step + 1)
                                    << " plan failed; -> SAFE_STOP";
                    _transition(State::SAFE_STOP, pose);
                    out.mode = Command::Mode::STOP;
                    break;
                }
                LOG("Mission") << "TEST: leg " << (_test_step + 1) << "/"
                               << TEST_WAYPOINT_COUNT << " -> ("
                               << wp.x << ", " << wp.y << ", "
                               << wp.theta << " rad)";
                _test_replan_needed = false;
                out.mode = Command::Mode::PATH_FOLLOW;
                break;
            }

            // Rising-edge: PurePursuit says we're at the leg goal. Latch
            // into the halt-and-localize hold before committing to advance.
            if (!_halting && _controller.has_arrived()) {
                const TestWaypoint& wp_cur = TEST_WAYPOINTS[_test_step];
                _halting       = true;
                _halt_started  = std::chrono::steady_clock::now();
                _halt_target_x = wp_cur.x;
                _halt_target_y = wp_cur.y;
                out.mode = Command::Mode::STOP;
                break;
            }

            if (_halting) {
                const auto elapsed_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - _halt_started).count();
                const double sigma_xy = std::sqrt(std::max(cov.xx, cov.yy));
                const bool min_done = elapsed_ms >= HALT_MIN_MS;
                const bool settled  = sigma_xy <= HALT_SIGMA_XY_TARGET_MM;
                const bool timed_out = elapsed_ms >= HALT_MAX_MS;
                if ((min_done && settled) || timed_out) {
                    const double mahal2 = mahalanobis2_xy(
                        pose, cov, _halt_target_x, _halt_target_y);
                    const bool gate_pass = (mahal2 <= MAHAL_GATE_CHI2);
                    LOG("Mission") << "TEST leg " << (_test_step + 1)
                                   << " halt done: " << elapsed_ms
                                   << "ms sigma_xy=" << sigma_xy
                                   << "mm mahal2=" << mahal2
                                   << (gate_pass ? " (pass)"
                                                 : " (FAIL — advancing anyway)");
                    _halting = false;
                    ++_test_step;
                    _test_replan_needed = true;
                }
                out.mode = Command::Mode::STOP;
                break;
            }

            out.mode = Command::Mode::PATH_FOLLOW;
            break;
        }

        case State::DONE:
        case State::SAFE_STOP:
        default:
            out.mode = Command::Mode::STOP;
            break;
    }

    // Top-level mission visibility on the dashboard. Snapshot the
    // mutex-protected fields under the lock before publishing — a plain
    // read of _approach_attempts (written by step itself and by start())
    // is a data race on ARM, and dashboard panels are not worth a torn
    // read. _search_skip_streak is single-thread-owned by step() but the
    // snapshot keeps it consistent with attempts.
    int attempts_now;
    bool skip_streak_now;
    {
        std::lock_guard<std::mutex> lk(_m);
        attempts_now    = _approach_attempts;
        skip_streak_now = _search_skip_streak;
    }
    dash.set("Mission", "state",      std::string(state_name(_state.load())));
    dash.set("Mission", "t_state_s",  t_in);
    dash.set("Mission", "box_to_home_mm", box_to_home);
    dash.set("Mission", "attempts",   static_cast<int64_t>(attempts_now));
    dash.set("Mission", "obstacles",
             static_cast<int64_t>(_obstacles.entries.size()));
    dash.set("Mission", "skipping",
             std::string(skip_streak_now ? "yes" : "no"));
    dash.set("Mission", "cmd_mode",
             std::string(out.mode == Command::Mode::PATH_FOLLOW ? "path" :
                         out.mode == Command::Mode::DIRECT      ? "direct" :
                                                                  "stop"));
    if (_ssearch.initialized) {
        const char* ph = "?";
        switch (_ssearch.phase) {
            case SimpleSearch::Phase::LOOK_LEFT:  ph = "L";   break;
            case SimpleSearch::Phase::LOOK_RIGHT: ph = "R";   break;
            case SimpleSearch::Phase::FORWARD:    ph = "FWD"; break;
        }
        // Append a R when the realign sub-state is active so the operator
        // can see the hysteresis at work (e.g. "FWD/R" during dodge
        // recovery vs plain "FWD" while driving).
        std::string ph_str(ph);
        if (_ssearch.in_realign) ph_str += "/R";
        dash.set("Mission", "search", ph_str);
        dash.set("Mission", "fwd_mm", _ssearch.forward_traveled_mm);
        dash.set("Mission", "obs_rot_deg",
                 _ssearch.obstacle_rotated_rad * 180.0 / M_PI);
    } else {
        dash.set("Mission", "search", std::string("-"));
    }

    return out;
}
