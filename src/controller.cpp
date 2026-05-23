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

// Squared Mahalanobis distance from `pose` (x, y) to (tx, ty) under the
// xy-block of `cov`. Returns +inf when the block is degenerate so the
// gate treats it as "wildly uncertain" rather than dividing by zero.
inline double mahalanobis2_xy(const Pos2D& pose, const Pose2DCov& cov,
                              double tx, double ty) {
    const double dx  = tx - pose.x;
    const double dy  = ty - pose.y;
    const double det = cov.xx * cov.yy - cov.xy * cov.xy;
    if (det <= 0.0) return std::numeric_limits<double>::infinity();
    return (cov.yy * dx * dx
            - 2.0 * cov.xy * dx * dy
            + cov.xx * dy * dy) / det;
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

void StateMachine::_transition(State new_state, const Pos2D& current) {
    const State old_state = _state.load();

    if (old_state == new_state) return;
    // Any state change clears halt-and-localize bookkeeping so an aborted
    // halt mid-state can't masquerade as a halt belonging to the next.
    _halting = false;
    // Exit side-effects on the outgoing state.
    if (old_state == State::CARRY_HOME || old_state == State::SEARCH) {
        LOG("CONTROLLER") << "Carried Home! Mission Accomplished";
        _controller.abort();
    }

    if (new_state == State::CARRY_HOME) {
        if (!_plan_path_home(current)) {
            new_state = State::SAFE_STOP;
        }
    }

    if (new_state == State::SEARCH) {
        // Fresh SEARCH session: previous skip-streak log is no longer
        // relevant; drop any latched APPROACH target; reset the simple
        // forward-then-rotate search state so the next entry starts a new
        // forward leg.
        _search_skip_streak    = false;
        _last_target_valid     = false;
        _ssearch.initialized   = false;
        servo_release();
    }

    if (new_state == State::APPROACH) {
        // Each approach starts from a fresh grip cycle.
        _grip_stop_sent = false;
        _grip_fired     = false;
    }

    if (new_state == State::RELEASE) {
        servo_release();
    }

    // No TURN_AROUND entry bookkeeping needed — REALIGN reads pose.theta
    // and _home_theta directly each tick.

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

bool StateMachine::_plan_path_home(const Pos2D& current) {
    // Snapshot mission config once.
    double hx, hy, ht, hm, arm;
    {
        std::lock_guard<std::mutex> lk(_m);
        hx  = _home_x; hy = _home_y; ht = _home_theta; hm = _home_margin;
        arm = _arm_length_mm;
    }

    // Bias the *robot* goal back by arm_length along the home heading so the
    // *box* (which sits arm_length ahead of the robot) ends up at home.
    const double gx = hx - arm * std::cos(ht);
    const double gy = hy - arm * std::sin(ht);

    // 3 control points: current pose, geometric midpoint, biased goal. The
    // midpoint gives the cubic basis enough shape to curve; with only two
    // controls the spline degenerates to a straight line and the start
    // tangent points straight at the goal regardless of current heading.
    PathPoint p_start;
    p_start.x = current.x;
    p_start.y = current.y;
    p_start.theta = current.theta;

    PathPoint p_mid;
    p_mid.x = 0.5 * (current.x + gx);
    p_mid.y = 0.5 * (current.y + gy);

    // Final heading = path tangent at the goal (midpoint -> goal). The 3
    // colinear controls collapse to a straight line, so this matches the
    // direction PURSUIT converges the robot to as it arrives. Setting the
    // goal theta to the tangent makes TERMINAL_ROTATE a no-op (delta ≈ 0),
    // eliminating the cosmetic final spin to home_theta. The post-release
    // REALIGN state restores the canonical home heading after backup.
    const double tangent_to_goal = std::atan2(gy - p_mid.y, gx - p_mid.x);

    PathPoint p_goal;
    p_goal.x = gx;
    p_goal.y = gy;
    p_goal.theta = tangent_to_goal;
    p_goal.margin = hm;

    std::vector<PathPoint> controls;
    controls.reserve(3);
    controls.push_back(p_start);
    controls.push_back(p_mid);
    controls.push_back(p_goal);

    std::vector<PathPoint> path = bcubic_spline(
        controls,
        30,
        40.0 * ROBOT_SPEED,
        40.0,
        100.0 * ROBOT_SPEED,
        30.0);

    if (path.empty()) {
        LOGE("Mission") << "bcubic_spline returned empty path "
                        << "from (" << current.x << ", " << current.y << ") "
                        << "to robot-goal (" << gx << ", " << gy << ")";
        return false;
    }

    path.back().margin = hm;
    path.back().theta  = tangent_to_goal;
    // Suppress unused-variable warning: ht (home_theta) is no longer baked
    // into the path; REALIGN handles heading reset after release.
    (void)ht;

    _controller.set_path(path);
    LOG("Mission") << "CARRY_HOME plan: " << path.size() << " pts, "
                   << "from (" << current.x << ", " << current.y << ") "
                   << "to robot-goal (" << gx << ", " << gy << ") "
                   << "[box -> (" << hx << ", " << hy << ")]";
    return true;
}

bool StateMachine::_plan_path_to(const Pos2D& current,
                                 double gx, double gy, double gtheta,
                                 double margin_mm, double v_max_mm_s) {
    PathPoint p_start;
    p_start.x = current.x;
    p_start.y = current.y;
    p_start.theta = current.theta;

    PathPoint p_mid;
    p_mid.x = 0.5 * (current.x + gx);
    p_mid.y = 0.5 * (current.y + gy);

    PathPoint p_goal;
    p_goal.x = gx;
    p_goal.y = gy;
    p_goal.theta = gtheta;
    p_goal.margin = margin_mm;

    std::vector<PathPoint> controls;
    controls.reserve(3);
    controls.push_back(p_start);
    controls.push_back(p_mid);
    controls.push_back(p_goal);

    std::vector<PathPoint> path = bcubic_spline(
        controls,
        30,
        v_max_mm_s,
        margin_mm,
        100.0 * ROBOT_SPEED,
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
    double hx, hy, hm, arm;
    int    attempts, max_attempts;
    {
        std::lock_guard<std::mutex> lk(_m);
        entered_at   = _state_entered;
        hx  = _home_x; hy = _home_y; hm = _home_margin;
        arm = _arm_length_mm;
        attempts     = _approach_attempts;
        max_attempts = _max_approach_attempts;
    }
    const double t_in    = seconds_since(entered_at);
    const State s = _state.load();

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

            const bool allow_engage = (t_in >= _search_min_dwell_s);
            const bool target_visible = (!snap.lost && snap.has_target);
            bool at_home_skip = false;

            // Vision has a target -> engage, but enforce the retry budget.
            if (allow_engage && target_visible) {
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
                    if (det_to_home < hm) {
                        if (!_search_skip_streak) {
                            LOG("Mission")
                                << "Skipping at-home box at world ("
                                << det_x << ", " << det_y << "), "
                                << det_to_home << " mm from home";
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
                    // hand off to APPROACH (or burn the budget).
                    _search_skip_streak = false;

                    if (attempts >= max_attempts) {
                        LOG("Mission") << "Approach budget exhausted ("
                                       << attempts << "/" << max_attempts << ")";
                        _transition(State::SAFE_STOP, pose);
                        out.mode = Command::Mode::STOP;
                        break;
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

            if (t_in > _search_timeout_s) {
                _transition(State::SAFE_STOP, pose);
                out.mode = Command::Mode::STOP;
                break;
            }

            if (!target_visible || !allow_engage) {
                _search_skip_streak = false;
            }

            // Very-simple search: forward _search_forward_target_mm, then
            // rotate 2π in place, repeat. Sidestep on lidar obstacle.
            if (!_ssearch.initialized) {
                _ssearch.phase = SimpleSearch::Phase::FORWARD;
                _ssearch.forward_traveled_mm  = 0.0;
                _ssearch.rotation_accumulated = 0.0;
                _ssearch.last_x     = pose.x;
                _ssearch.last_y     = pose.y;
                _ssearch.last_theta = pose.theta;
                _ssearch.initialized = true;
            }

            if (_ssearch.phase == SimpleSearch::Phase::FORWARD) {
                // Obstacle in the forward arc -> rotate in place to clear
                // it. Accumulate nothing on the forward odometer; the leg
                // resumes once the path is open.
                if (app::obstacle_ahead.load()) {
                    _ssearch.last_x     = pose.x;
                    _ssearch.last_y     = pose.y;
                    _ssearch.last_theta = pose.theta;
                    out.mode = Command::Mode::DIRECT;
                    out.v_mm_s = 0.0;
                    out.omega_rad_s = _search_omega;
                    break;
                }
                const double dx = pose.x - _ssearch.last_x;
                const double dy = pose.y - _ssearch.last_y;
                _ssearch.forward_traveled_mm += std::hypot(dx, dy);
                _ssearch.last_x     = pose.x;
                _ssearch.last_y     = pose.y;
                _ssearch.last_theta = pose.theta;

                if (_ssearch.forward_traveled_mm >= _search_forward_target_mm) {
                    _ssearch.phase = SimpleSearch::Phase::ROTATE;
                    _ssearch.rotation_accumulated = 0.0;
                    _ssearch.last_theta = pose.theta;
                    out.mode = Command::Mode::DIRECT;
                    out.v_mm_s = 0.0;
                    out.omega_rad_s = _search_omega;
                    break;
                }
                out.mode = Command::Mode::DIRECT;
                out.v_mm_s = _search_v_forward_mm_s;
                out.omega_rad_s = 0.0;
                break;
            }

            // Phase::ROTATE: spin in place until ~2π accumulated, then start
            // a new forward leg.
            const double dth = std::abs(wrap_pi(pose.theta - _ssearch.last_theta));
            _ssearch.rotation_accumulated += dth;
            _ssearch.last_theta = pose.theta;
            if (_ssearch.rotation_accumulated >= 2.0 * M_PI) {
                _ssearch.phase = SimpleSearch::Phase::FORWARD;
                _ssearch.forward_traveled_mm = 0.0;
                _ssearch.last_x = pose.x;
                _ssearch.last_y = pose.y;
            }
            out.mode = Command::Mode::DIRECT;
            out.v_mm_s = 0.0;
            out.omega_rad_s = _search_omega;
            break;
        }

        case State::APPROACH: {
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

            // 2. Pre-grip captured (rare: e.g. the box rolls into the
            //    retention ROI before the grab gate triggers). Still go via
            //    CARRY_HOME unconditionally so we never instant-release.
            if (snap.captured) {
                { std::lock_guard<std::mutex> lk(_m); _approach_attempts = 0; }
                _transition(State::CARRY_HOME, pose);
                out.mode = Command::Mode::PATH_FOLLOW;
                break;
            }

            // 3. Box is in the grab zone. Two-tick sequence so the motor
            //    has a control cycle to decelerate before the gripper
            //    target is latched:
            //      tick N   — command v=0, latch _grip_stop_sent
            //      tick N+1 — call servo_grab() (non-blocking: latches the
            //                 target angle; servo_tick() in loop_control
            //                 walks the PWM in STEP_DEG / STEP_MS chunks to
            //                 cap inrush current). _grip_fired is set
            //                 immediately, and v=0 stays commanded by the
            //                 hold-after-grab logic below while the slew
            //                 completes.
            if (snap.grabbable) {
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

            if (!target_visible) {
                // Vision dropped. If we still have a latched world target
                // and we haven't reached it yet, keep driving toward it
                // instead of spinning — re-entering SEARCH here was the
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

            // Visual servo: forward at constant speed, steer with bearing.
            // Bearing is +left (tracker convention); motor omega is
            // CCW-positive, which matches: +bearing -> +omega.
            out.mode = Command::Mode::DIRECT;
            out.v_mm_s = _approach_v;
            out.omega_rad_s = clamp_d(_approach_k_omega * snap.bearing_rad,
                                      -_approach_omega_max,
                                      _approach_omega_max);
            break;
        }

        case State::CARRY_HOME: {
            // We do NOT bail to SEARCH on !snap.captured here. Once the
            // gripper closes, the arms occlude the box and the tracker
            // routinely loses the lock — the prior bail caused
            // CARRY_HOME → SEARCH → servo_release() → re-grab → repeat
            // oscillation that looked like grabs/releases during turns.
            // The drive-home plan is committed; _carry_timeout_s caps it.
            if (_controller.has_arrived()) {
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

            // No periodic re-plan: the 3-point spline collapses to a straight
            // line, and set_path() would reset PurePursuit to PRE_ROLL —
            // causing visible stop-rotate-go thrash every cycle. KF + cox
            // corrections plus PurePursuit's cross-track curvature term
            // handle pose drift continuously without a fresh plan.
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
            // REALIGN: rotate in place to the mission-start home_theta. The
            // robot is at the back-off position (RELEASE has just driven it
            // ~80 mm behind the delivered box) and we want the wall closest
            // to home to stay behind the robot — so the next SEARCH FORWARD
            // leg drives into the arena, not into the wall. The at-home
            // filter in SEARCH skips the just-delivered box even if it falls
            // back into the camera frame after this rotation.
            double home_theta_local;
            { std::lock_guard<std::mutex> lk(_m); home_theta_local = _home_theta; }
            const double delta = wrap_pi(home_theta_local - pose.theta);
            if (std::fabs(delta) < 0.05) {  // ~3°
                _transition(State::SEARCH, pose);
                out.mode = Command::Mode::STOP;
                break;
            }
            // Hard timeout so a jammed motor / blocked chassis can't wedge
            // here forever. Bailing into SEARCH means the mission's own
            // timeouts still apply and we can either retry or eventually
            // reach SAFE_STOP.
            if (t_in > _turn_around_timeout_s) {
                LOG("Mission") << "REALIGN timed out after " << t_in
                               << " s with " << (delta * 180.0 / M_PI)
                               << " deg remaining; bailing to SEARCH.";
                _transition(State::SEARCH, pose);
                out.mode = Command::Mode::STOP;
                break;
            }
            out.mode = Command::Mode::DIRECT;
            out.v_mm_s = 0.0;
            out.omega_rad_s = clamp_d(2.0 * delta,
                                      -_search_omega, _search_omega);
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
                vmax = _goto_v_max_mm_s; margin = _goto_margin_mm;
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
                    _transition(State::DONE, pose);
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
                                   _test_margin_mm, _test_v_max_mm_s)) {
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
    dash.set("Mission", "skipping",
             std::string(skip_streak_now ? "yes" : "no"));
    dash.set("Mission", "cmd_mode",
             std::string(out.mode == Command::Mode::PATH_FOLLOW ? "path" :
                         out.mode == Command::Mode::DIRECT      ? "direct" :
                                                                  "stop"));
    if (_ssearch.initialized) {
        dash.set("Mission", "search",
                 std::string(_ssearch.phase == SimpleSearch::Phase::FORWARD
                                 ? "FWD" : "ROT"));
        dash.set("Mission", "fwd_mm", _ssearch.forward_traveled_mm);
        dash.set("Mission", "rot_deg",
                 _ssearch.rotation_accumulated * 180.0 / M_PI);
    } else {
        dash.set("Mission", "search", std::string("-"));
    }

    return out;
}
