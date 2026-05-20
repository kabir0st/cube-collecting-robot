#include <controller.hpp>

#include <algorithm>
#include <cmath>
#include <vector>
#include <string>

#include <calibrations.hpp>
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
    }
    return "?";
}

StateMachine::StateMachine(TrackerResult& vision,
                           PurePursuitController& controller)
    : _vision(vision), _controller(controller) {
    const auto now = std::chrono::steady_clock::now();
    _state_entered = now;
    _last_replan_at = now;
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
    LOG("Mission") << "start(): " << state_name(now) << " -> SEARCH";
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
        // relevant; let the first at-home detection log again. Drop any
        // latched APPROACH target so the next approach starts from a
        // fresh vision lock. Invalidate the waypoint plan so it regenerates
        // — this is what enables multi-box re-search from a new pose.
        _search_skip_streak = false;
        _last_target_valid  = false;
        _search_path_active = false;
        _search_last_wp_idx = static_cast<size_t>(-1);
        _search_plan.valid  = false;
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

    if (new_state == State::TURN_AROUND) {
        _turn_start_set   = false;
        _turn_accumulated = 0.0;
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
        _last_replan_at = now;        // (CARRY_HOME entry just re-planned)
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

    PathPoint p_goal;
    p_goal.x = gx;
    p_goal.y = gy;
    p_goal.theta = ht;
    p_goal.margin = hm;

    std::vector<PathPoint> controls;
    controls.reserve(3);
    controls.push_back(p_start);
    controls.push_back(p_mid);
    controls.push_back(p_goal);

    std::vector<PathPoint> path = bcubic_spline(
        controls,
        30,
        80.0,
        40.0,
        200.0,
        30.0);

    if (path.empty()) {
        LOGE("Mission") << "bcubic_spline returned empty path "
                        << "from (" << current.x << ", " << current.y << ") "
                        << "to robot-goal (" << gx << ", " << gy << ")";
        return false;
    }

    path.back().margin = hm;
    path.back().theta  = ht;

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
        200.0,
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

void StateMachine::_build_search_plan(const Pos2D& /*pose*/) {
    calib::ArenaBounds ab;
    if (!calib::get_arena_bounds(ab)) {
        _search_plan.valid = false;
        return;
    }
    const double min_x = ab.min_x + _waypoint_margin_mm;
    const double max_x = ab.max_x - _waypoint_margin_mm;
    const double min_y = ab.min_y + _waypoint_margin_mm;
    const double max_y = ab.max_y - _waypoint_margin_mm;
    if (max_x <= min_x || max_y <= min_y) {
        // Margin ate the bounds — arena too small to plan in. Spin and hope
        // the box wanders into view.
        _search_plan.valid = false;
        return;
    }
    const double W    = max_x - min_x;
    const double H    = max_y - min_y;
    const double step = 2.0 * _box_detection_range_mm;
    const int nx = std::max(1, (int)std::ceil(W / step));
    const int ny = std::max(1, (int)std::ceil(H / step));

    _search_plan.waypoints.clear();
    _search_plan.waypoints.reserve(static_cast<size_t>(nx * ny) + 1);
    // Center first so a small arena resolves in one rotation.
    _search_plan.waypoints.emplace_back(0.5 * (min_x + max_x),
                                        0.5 * (min_y + max_y));
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const double wx = min_x + (i + 0.5) * W / nx;
            const double wy = min_y + (j + 0.5) * H / ny;
            _search_plan.waypoints.emplace_back(wx, wy);
        }
    }
    _search_plan.idx                  = 0;
    _search_plan.at_waypoint          = false;
    _search_plan.accumulated_rotation = 0.0;
    _search_plan.valid                = true;
    LOG("Mission") << "Search plan: " << _search_plan.waypoints.size()
                   << " waypoints (" << nx << "x" << ny << " grid + center)";
}

StateMachine::Command StateMachine::step(const Pos2D& pose, Dashboard& dash) {
    Command out;

    // Snapshot everything once so the state body sees a consistent view.
    // One mutex acquire per tick on each of _m and the vision lock
    const TrackerResult::Snapshot snap = _vision.get_snapshot();
    std::chrono::steady_clock::time_point entered_at;
    std::chrono::steady_clock::time_point last_replan;
    double hx, hy, hm, arm;
    int    attempts, max_attempts;
    {
        std::lock_guard<std::mutex> lk(_m);
        entered_at   = _state_entered;
        last_replan  = _last_replan_at;
        hx  = _home_x; hy = _home_y; hm = _home_margin;
        arm = _arm_length_mm;
        attempts     = _approach_attempts;
        max_attempts = _max_approach_attempts;
    }
    const double t_in    = seconds_since(entered_at);
    const double t_since_replan = seconds_since(last_replan);
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

            // Adaptive-subdivision waypoint walker. Plan is rebuilt on every
            // SEARCH entry (see _transition). If the arena bounds aren't
            // published yet we just spin in place until they are.
            if (!_search_plan.valid) {
                _build_search_plan(pose);
            }
            if (!_search_plan.valid) {
                out.mode = Command::Mode::DIRECT;
                out.v_mm_s = 0.0;
                out.omega_rad_s = _search_omega;
                break;
            }

            if (_search_plan.idx >= _search_plan.waypoints.size()) {
                // Visited every waypoint without seeing a box. Mission done.
                _transition(State::DONE, pose);
                out.mode = Command::Mode::STOP;
                break;
            }

            const double wx = _search_plan.waypoints[_search_plan.idx].first;
            const double wy = _search_plan.waypoints[_search_plan.idx].second;

            if (!_search_plan.at_waypoint) {
                const double dist = std::hypot(wx - pose.x, wy - pose.y);
                const bool arrived = (dist <= _waypoint_margin_mm) ||
                                     (_search_path_active && _controller.has_arrived());
                if (arrived) {
                    _search_plan.at_waypoint = true;
                    _search_plan.rotation_start_theta = pose.theta;
                    _search_plan.accumulated_rotation = 0.0;
                    _search_path_active = false;
                    _controller.abort();
                } else {
                    if (!_search_path_active || _search_last_wp_idx != _search_plan.idx) {
                        const double gtheta = std::atan2(wy - pose.y, wx - pose.x);
                        if (_plan_path_to(pose, wx, wy, gtheta,
                                          _waypoint_margin_mm,
                                          _search_v_max_mm_s)) {
                            _search_path_active = true;
                            _search_last_wp_idx = _search_plan.idx;
                        } else {
                            _search_path_active = false;
                        }
                    }
                    out.mode = _search_path_active ? Command::Mode::PATH_FOLLOW
                                                   : Command::Mode::STOP;
                    break;
                }
            }

            // at_waypoint == true: rotate in place until ~2π accumulated.
            const double dtheta = std::abs(wrap_pi(pose.theta - _search_plan.rotation_start_theta));
            _search_plan.accumulated_rotation += dtheta;
            _search_plan.rotation_start_theta = pose.theta;
            if (_search_plan.accumulated_rotation >= 2.0 * M_PI) {
                _search_plan.idx++;
                _search_plan.at_waypoint = false;
                _search_plan.accumulated_rotation = 0.0;
                _search_last_wp_idx = static_cast<size_t>(-1);
            }
            out.mode = Command::Mode::DIRECT;
            out.v_mm_s = 0.0;
            out.omega_rad_s = _search_omega;
            break;
        }

        case State::APPROACH: {
            // 1. Vision tracker latched a hold (10-frame debounce on grip ROI).
            //    Hand off to carry/release.
            if (snap.captured) {
                { std::lock_guard<std::mutex> lk(_m); _approach_attempts = 0; }
                // Already at home? Skip the carry phase. Decision uses the
                // BOX position (pose + arm), not the robot pose, so that we
                // do not back up with the box still parked in the gripper
                // outside the home zone.
                if (box_to_home < hm) {
                    _transition(State::RELEASE, pose);
                    out.mode = Command::Mode::DIRECT;
                    out.v_mm_s = _release_v;
                    out.omega_rad_s = 0.0;
                } else {
                    _transition(State::CARRY_HOME, pose);
                    out.mode = Command::Mode::PATH_FOLLOW;
                }
                break;
            }

            // 2. Servo already fired — sit still and wait for `captured` to
            //    latch. If it never does within _grip_settle_s the close
            //    didn't catch; re-open, burn an attempt, go back to SEARCH.
            if (_grip_fired) {
                if (seconds_since(_grip_fired_at) > _grip_settle_s) {
                    servo_release();
                    _grip_fired = false;
                    { std::lock_guard<std::mutex> lk(_m); ++_approach_attempts; }
                    _transition(State::SEARCH, pose);
                    out.mode = Command::Mode::STOP;
                    break;
                }
                out.mode = Command::Mode::DIRECT;
                out.v_mm_s = 0.0;
                out.omega_rad_s = 0.0;
                break;
            }

            // 3. Box is in the grab zone. Two-tick sequence so the motor
            //    has a control cycle to decelerate before the blocking
            //    smooth-close runs:
            //      tick N   — command v=0, latch _grip_stop_sent
            //      tick N+1 — call servo_grab() (blocks ~1.2 s)
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
                // This is a real failed attempt — charge the budget here.
                { std::lock_guard<std::mutex> lk(_m); ++_approach_attempts; }
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
            if (!snap.captured) {
                // Box slipped out of the grip mid-drag (debounced by the
                // tracker's retention-ROI hysteresis, so single-frame
                // glitches don't trigger this).
                _transition(State::SEARCH, pose);
                out.mode = Command::Mode::DIRECT;
                out.v_mm_s = 0.0;
                out.omega_rad_s = _search_omega;
                break;
            }

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

            // Periodic re-plan so a Kalman jump (Cox correction) doesn't
            // leave us trying to follow a stale spline. Reuses
            // _plan_path_home, which rebuilds from the *current* pose.
            //
            // Skip when the controller has already reached the goal position
            // (TERMINAL_ROTATE / DONE): re-planning there would reset the
            // closest-index hint and kick the controller back to PRE_ROLL /
            // PURSUIT, wasting the alignment work it just did.
            if (t_since_replan > _carry_replan_s && !_controller.is_near_goal()) {
                if (!_plan_path_home(pose)) {
                    _transition(State::SAFE_STOP, pose);
                    out.mode = Command::Mode::STOP;
                    break;
                }
                std::lock_guard<std::mutex> lk(_m);
                _last_replan_at = std::chrono::steady_clock::now();
            }

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
            // 180° in-place pivot before re-entering SEARCH, so the camera
            // is pointing away from the just-delivered box (and the at-home
            // filter has a chance to skip it cleanly when it does come into
            // frame again).
            if (!_turn_start_set) {
                _turn_start_theta = pose.theta;
                _turn_accumulated = 0.0;
                _turn_start_set   = true;
                out.mode = Command::Mode::DIRECT;
                out.v_mm_s = 0.0;
                out.omega_rad_s = _search_omega;
                break;
            }
            const double dtheta = std::abs(wrap_pi(pose.theta - _turn_start_theta));
            _turn_accumulated += dtheta;
            _turn_start_theta = pose.theta;
            if (_turn_accumulated >= M_PI) {
                _transition(State::SEARCH, pose);
                out.mode = Command::Mode::STOP;
                break;
            }
            out.mode = Command::Mode::DIRECT;
            out.v_mm_s = 0.0;
            out.omega_rad_s = _search_omega;
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
    dash.set("Mission", "search_wp",
             _search_plan.valid
                 ? (std::to_string(_search_plan.idx) + "/" +
                    std::to_string(_search_plan.waypoints.size()))
                 : std::string("-"));

    return out;
}
