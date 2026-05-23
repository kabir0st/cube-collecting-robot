#pragma once

#include <cstddef>
#include <mutex>
#include <vector>

#include <global.hpp>
#include <types.hpp>

// Four internal modes:
//   PRE_ROLL
//   PURSUIT 
//   TERMINAL_ROTATE 
//   DONE
class PurePursuitController {
public:
    PurePursuitController();
    void set_path(const std::vector<PathPoint>& path);
    void abort();
    bool has_arrived() const;
    bool is_near_goal() const;

    // Dashboard helpers: where in the path the controller is currently
    // aiming and how long the path is. Used only for telemetry.
    size_t   current_index() const;
    size_t   total() const;

    // The point the controller is currently steering toward — lookahead point
    // in PURSUIT, last point in TERMINAL_ROTATE/DONE. Returned as a PathPoint
    // copy so the caller (dashboard, debug) sees the same fields the path
    // planner produced.
    PathPoint get_active_target() const;

    // True iff `get_active_target()` reflects a real path target rather than
    // a default-constructed PathPoint{}. False before the first set_path()
    // call, after abort(), and while idling. Telemetry consumers gate
    // their "draw target" overlay on this.
    bool has_active_target() const;

    // Snapshot of the currently-loaded path.
    std::vector<PathPoint> get_path() const;

    // One control step. Reads pose, decides mode, writes target (v, ω).
    void compute(const Pos2D& pose,
                 double& v_mm_s, double& omega_rad_s);

private:
    enum class Mode { IDLE, PRE_ROLL, PURSUIT, TERMINAL_ROTATE, DONE };

    mutable std::mutex      _m;
    std::vector<PathPoint>  _path;
    size_t                  _closest_idx = 0;   // monotonic hint into _path
    Mode                    _mode = Mode::IDLE;
    bool                    _arrived = true;

    // Last point we actually steered toward — published as the "current
    // target" for the dashboard.
    PathPoint _active_target{};

    // Tunables.
    double _L_d = 250.0;  // mm, lookahead distance
    double _v_max = 75.0 * ROBOT_SPEED;   // mm/s, absolute clamp
    double _omega_max = 1.5 * ROBOT_SPEED;    // rad/s, absolute clamp
    double _heading_done_rad = 0.05;   // ~3 deg, TERMINAL_ROTATE exit
    double _pre_roll_thresh = 0.52;   // ~30 deg, PRE_ROLL entry
    double _k_pre_roll = 2.0;    // P gain for PRE_ROLL omega target
    double _k_terminal = 2.0;    // P gain for terminal rotate omega
    // In-place rotation cap. PRE_ROLL and TERMINAL_ROTATE both spin the
    // chassis with v=0, which dominates the wheel-slip portion of the
    // odometry noise (var_theta ~ K_ROT^2 * d_theta^2). At the full
    // _omega_max (~5.4 rad/s) a 180° turn finishes in ~0.6s — faster than
    // cox can fold in enough corrections to bound heading drift. Capping
    // these phases to 1.0 rad/s spreads the rotation over several seconds
    // so cox provides continuous heading anchoring while the wheels move.
    double _inplace_omega_cap = 1.0;     // rad/s

    // Walk forward from `_closest_idx` along the path, advancing the hint
    // past any points we've already passed (cumulative arc length is what
    // separates "in front" from "behind"). Returns the index of the
    // lookahead point at distance >= `_L_d` ahead of the closest point.
    // Caller must hold `_m`.
    size_t _find_lookahead_index_locked(const Pos2D& pose);

    // Pure Pursuit geometry: transform `target` to the robot frame and emit
    // the (v, omega) the steering law commands. Used by both PURSUIT and
    // the PRE_ROLL -> PURSUIT fallthrough so the formula lives in one place.
    // Caller must hold `_m`.
    void _compute_pursuit_locked(const Pos2D& pose,
                                 const PathPoint& target,
                                 double& v_target,
                                 double& w_target) const;
};
