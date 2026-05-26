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

    // Dashboard helpers where in the path the controller is currently
    // aiming and how long the path is. Used only for telemetry.
    size_t   current_index() const;
    size_t   total() const;

    // This is the point the controller is currently steering toward the lookahead point
    // in PURSUIT, last point in TERMINAL_ROTATE/DONE. Returned as a PathPoint
    // copy so the caller (dashboard, debug) sees the same fields the path
    // planner produced.
    PathPoint get_active_target() const;

    bool has_active_target() const;
    std::vector<PathPoint> get_path() const;

    // One control step. Reads pose, decides mode, writes target (v, ω).
    void compute(const Pos2D& pose,
                 double& v_mm_s, double& omega_rad_s);

private:
    enum class Mode { IDLE, PRE_ROLL, PURSUIT, TERMINAL_ROTATE, DONE };

    mutable std::mutex      _m;
    std::vector<PathPoint>  _path;
    size_t                  _closest_idx = 0;
    Mode                    _mode = Mode::IDLE;
    bool                    _arrived = true;

    PathPoint _active_target{};

    // mm, lookahead distance
    double _lookahead_dist = 250.0;  
    double _v_max = 75.0 * ROBOT_SPEED;   
    double _omega_max = 2.0; 
    double _heading_done_rad = 0.05;
    double _terminal_unlatch_extra_mm = 30.0;   // mm
    double _pre_roll_thresh = 0.52;
    double _inplace_k_omega = 2.0;
    double _inplace_omega_cap = 0.8;     // rad/s

    size_t _find_lookahead_index_locked(const Pos2D& pose);

    void _compute_pursuit_locked(const Pos2D& pose,
                                 const PathPoint& target,
                                 double& v_target,
                                 double& w_target) const;
};
