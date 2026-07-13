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

bool PurePursuitController::has_active_target() const {
    std::lock_guard<std::mutex> lk(_m);
    return _mode != Mode::IDLE && !_path.empty();
}

std::vector<PathPoint> PurePursuitController::get_path() const {
    std::lock_guard<std::mutex> lk(_m);
    return _path;
}

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
    while (idx + 1 < _path.size() && accumulated < _lookahead_dist) {
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
    const double dx = target.x - pose.x;
    const double dy = target.y - pose.y;
    // Lateral offset of the lookahead in the robot's body frame.
    // Positive = lookahead is to the robot's left -> turn left (+omega).
    const double lookahead_lateral =
        -dx * std::sin(pose.theta) + dy * std::cos(pose.theta);
    // Using chord squared
    const double chord_sq = dx*dx + dy*dy;
    const double curvature =
        (chord_sq > 1e-6) ? (2.0 * lookahead_lateral) / chord_sq : 0.0;
    v_target = std::min(target.v_max, _v_max);
    w_target = v_target * curvature;
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

        // Re-evaluate TERMINAL_ROTATE every tick
        if (_mode != Mode::DONE) {
            const bool at_last_waypoint = (_closest_idx + 1 >= _path.size());
            if (_mode == Mode::TERMINAL_ROTATE) {
                const double unlatch_radius =
                    last.margin + _terminal_unlatch_extra_mm;
                if (!at_last_waypoint || rho_last > unlatch_radius) {
                    _mode = Mode::PURSUIT;
                }
            } else if (at_last_waypoint && rho_last < last.margin) {
                _mode = Mode::TERMINAL_ROTATE;
                _active_target = last;
            }
        }

        switch (_mode) {
            case Mode::PRE_ROLL: {
                // Pick the lookahead first so we know which way "forward"
                // points and if the heading error to that bearing is small,
                // engage PURSUIT immediately
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
                    // Proportional to target but capped to _inplace_omega_cap
                    // (gentler than _omega_max) so cox has time to fold
                    // corrections into the KF heading during the spin.
                    v_target = 0.0;
                    w_target = std::max(-_inplace_omega_cap,
                              std::min( _inplace_omega_cap,
                                        _inplace_k_omega * err));
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
                    // Same in-place omega cap as PRE_ROLL TERMINAL_ROTATE
                    // is the symmetric end-of-path version of the same
                    // wheel-slip-dominant scenario.
                    v_target = 0.0;
                    w_target = std::max(-_inplace_omega_cap,
                              std::min( _inplace_omega_cap,
                                        _inplace_k_omega * delta));
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

    v_target = std::max(-_v_max,     std::min(v_target, _v_max));
    w_target = std::max(-_omega_max, std::min(w_target, _omega_max));

    v_mm_s      = v_target;
    omega_rad_s = w_target;
}
