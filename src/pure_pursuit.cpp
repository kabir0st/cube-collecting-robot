#include <pure_pursuit.hpp>

#include <algorithm>
#include <cmath>
#include <utils.hpp>
#include <telemetry.hpp>

namespace {
    inline PathPoint default_target() {
        PathPoint p;
        p.x = 0.0; p.y = 0.0; p.theta = 0.0;
        p.v_max = 0.0; p.margin = 0.0;
        return p;
    }
}

PurePursuitController::PurePursuitController() = default;

void PurePursuitController::set_path(const std::vector<PathPoint>& path) {
    {
        std::lock_guard<std::mutex> lk(_m);
        _path = path;
        _closest_idx = 0;
        _active_target = _path.empty() ? default_target() : _path.front();
        if (_path.empty()) {
            _mode = Mode::IDLE;
            _arrived = true;
        } else {
            // Defer the PRE_ROLL-vs-PURSUIT choice to the first compute() — we
            // need a pose to know the heading error.
            _mode = Mode::PRE_ROLL;
            _arrived = false;
        }
    }
    telemetry::send_path(path);
}

void PurePursuitController::abort() {
    std::lock_guard<std::mutex> lk(_m);
    _mode = Mode::IDLE;
    _arrived = true;
}

bool PurePursuitController::has_arrived() const {
    std::lock_guard<std::mutex> lk(_m);
    return _arrived;
}

bool PurePursuitController::is_near_goal() const {
    std::lock_guard<std::mutex> lk(_m);
    return _mode == Mode::TERMINAL_ROTATE || _mode == Mode::DONE;
}

size_t PurePursuitController::current_index() const {
    std::lock_guard<std::mutex> lk(_m);
    return _closest_idx;
}

size_t PurePursuitController::total() const {
    std::lock_guard<std::mutex> lk(_m);
    return _path.size();
}

PathPoint PurePursuitController::get_active_target() const {
    std::lock_guard<std::mutex> lk(_m);
    return _active_target;
}

std::vector<PathPoint> PurePursuitController::get_path() const {
    std::lock_guard<std::mutex> lk(_m);
    return _path;
}

// Two-phase walker, no robot-offset seeding.
//
// 1. Window-scan: pick the absolute-minimum-distance point in a forward
//    window of CLOSEST_SCAN_WINDOW samples starting at the current
//    _closest_idx. Monotonic — we never search behind _closest_idx because
//    Pure Pursuit only moves forward along the path. More robust than a
//    "next closer than current" heuristic, which can stall on tight curves
//    where adjacent path points are at comparable distances.
//
//    Window size: at v_max=150 mm/s and dt=0.2 s the robot advances at most
//    ~30 mm/tick; B-spline samples here are spaced ~17-34 mm; so 10 points
//    covers a generous worst case.
//
// 2. From the (new) closest index, accumulate path arc length until it
//    reaches _L_d. Cross-track recovery — when the robot is off the path
//    — happens naturally via the y_l term in the curvature formula, so we
//    do NOT add the robot->closest distance to the accumulator (doing so
//    causes a Cox-jump-induced pose offset to collapse the lookahead onto
//    the closest point, which is the opposite of what Pure Pursuit needs
//    in that situation).
size_t PurePursuitController::_find_lookahead_index_locked(const Pos2D& pose) {
    if (_path.empty()) return 0;

    constexpr size_t CLOSEST_SCAN_WINDOW = 10;
    size_t best_idx  = _closest_idx;
    double best_dsq;
    {
        const double dx = _path[_closest_idx].x - pose.x;
        const double dy = _path[_closest_idx].y - pose.y;
        best_dsq = dx*dx + dy*dy;
    }
    const size_t window_end =
        std::min(_closest_idx + CLOSEST_SCAN_WINDOW, _path.size());
    for (size_t i = _closest_idx + 1; i < window_end; ++i) {
        const double dx  = _path[i].x - pose.x;
        const double dy  = _path[i].y - pose.y;
        const double dsq = dx*dx + dy*dy;
        if (dsq < best_dsq) {
            best_dsq = dsq;
            best_idx = i;
        }
    }
    _closest_idx = best_idx;

    double accumulated = 0.0;
    size_t idx = _closest_idx;
    while (idx + 1 < _path.size() && accumulated < _L_d) {
        const double dx = _path[idx+1].x - _path[idx].x;
        const double dy = _path[idx+1].y - _path[idx].y;
        accumulated += std::hypot(dx, dy);
        ++idx;
    }
    return idx;
}

void PurePursuitController::_compute_pursuit_locked(const Pos2D& pose,
                                                    const PathPoint& target,
                                                    double& v_target,
                                                    double& w_target) const {
    const double dx   = target.x - pose.x;
    const double dy   = target.y - pose.y;
    // y_l: cross-track component of the lookahead in the robot frame.
    // +y_l means lookahead is to the robot's left -> turn left (+omega).
    const double y_l  = -dx * std::sin(pose.theta) + dy * std::cos(pose.theta);
    // Use the ACTUAL chord squared (dx^2 + dy^2), not _L_d^2. Near the end
    // of the path the lookahead clamps to the last point at chord < _L_d;
    // using _L_d in the denominator under-estimates curvature exactly when
    // the robot most needs to commit to the final approach.
    const double L_sq = dx*dx + dy*dy;
    const double kappa = (L_sq > 1e-6) ? (2.0 * y_l) / L_sq : 0.0;
    v_target = std::min(target.v_max, _v_max);
    w_target = v_target * kappa;
}

void PurePursuitController::compute(const Pos2D& pose,
                                    double& v_mm_s,
                                    double& omega_rad_s) {
    std::lock_guard<std::mutex> lk(_m);

    double v_target = 0.0;
    double w_target = 0.0;

    if (_mode != Mode::IDLE && !_path.empty()) {
        const PathPoint& last = _path.back();
        const double dx_last = last.x - pose.x;
        const double dy_last = last.y - pose.y;
        const double rho_last = std::hypot(dx_last, dy_last);

        // Re-evaluate TERMINAL_ROTATE every tick (DONE is sticky — once
        // aligned, we stop).
        // Two guards on entry:
        //   - `_closest_idx == last`: we actually traversed the path. Stops
        //     accidental entry when a path doubles back through the start.
        //   - `rho_last < margin`: physically at the goal.
        // Un-latch back to PURSUIT if we drift outside margin (inertia, a Cox
        // correction jump). Re-evaluating each tick means a robot that
        // drifts past the goal can recover by pursuing the path back to it,
        // instead of rotating in place far from where it should be.
        if (_mode != Mode::DONE) {
            const bool at_last_waypoint = (_closest_idx + 1 >= _path.size());
            const bool within_margin    = (rho_last < last.margin);
            if (at_last_waypoint && within_margin) {
                _mode = Mode::TERMINAL_ROTATE;
                _active_target = last;
            } else if (_mode == Mode::TERMINAL_ROTATE) {
                // Drifted out — re-engage pursuit. Don't go back to PRE_ROLL
                // because the robot is already moving and roughly aligned;
                // PURSUIT's curvature will pull it back to the path.
                _mode = Mode::PURSUIT;
            }
        }

        switch (_mode) {
            case Mode::PRE_ROLL: {
                // Pick the lookahead first so we know which way "forward"
                // points; if the heading error to that bearing is small,
                // engage PURSUIT immediately (same tick, no zero-output stall).
                const size_t look_idx = _find_lookahead_index_locked(pose);
                _active_target = _path[look_idx];
                const double dx = _active_target.x - pose.x;
                const double dy = _active_target.y - pose.y;
                const double bearing = std::atan2(dy, dx);
                const double err = wrap_pi(bearing - pose.theta);
                if (std::fabs(err) < _pre_roll_thresh) {
                    _mode = Mode::PURSUIT;
                    _compute_pursuit_locked(pose, _active_target,
                                            v_target, w_target);
                } else {
                    // Rotate in place toward the lookahead bearing.
                    // Proportional target — the Motor smoother shapes
                    // acceleration; the ω_max clamp below caps amplitude.
                    v_target = 0.0;
                    w_target = _k_pre_roll * err;
                }
                break;
            }

            case Mode::PURSUIT: {
                const size_t look_idx = _find_lookahead_index_locked(pose);
                _active_target = _path[look_idx];
                _compute_pursuit_locked(pose, _active_target,
                                        v_target, w_target);
                break;
            }

            case Mode::TERMINAL_ROTATE: {
                const double delta = wrap_pi(last.theta - pose.theta);
                if (std::fabs(delta) < _heading_done_rad) {
                    _mode = Mode::DONE;
                    _arrived = true;
                    v_target = 0.0;
                    w_target = 0.0;
                } else {
                    v_target = 0.0;
                    w_target = _k_terminal * delta;
                }
                break;
            }

            case Mode::DONE:
            case Mode::IDLE:
            default:
                v_target = 0.0;
                w_target = 0.0;
                break;
        }
    }

    // Saturate to absolute limits. Slew shaping and any low-pass filtering
    // live downstream in the Motor's 50 Hz IO loop, so this function emits
    // the *target* (v, ω) — staircase-shaped at the 5 Hz control rate, but
    // interpolated by the motor smoother into a continuous chassis command.
    v_target = std::max(-_v_max,     std::min(v_target, _v_max));
    w_target = std::max(-_omega_max, std::min(w_target, _omega_max));

    v_mm_s      = v_target;
    omega_rad_s = w_target;
}
