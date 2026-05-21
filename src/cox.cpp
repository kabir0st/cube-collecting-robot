#include <vector>
#include <cmath>
#include <map>
#include <cstring>
#include <limits>

#include <calibrations.hpp>
#include <cox.hpp>
#include <types.hpp>
#include <utils.hpp>
#include <global.hpp>
#include <loggers.hpp>
#include <thread>
#include <iostream>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <dashboard.hpp>
#include <kalman.hpp>

namespace {

    const float ENDPOINT_SLACK_MM = 50.0f;     // absolute slack along segment
    const float MAX_GATE_MM = 150.0f;
    const float MIN_GATE_MM  = 8.0f;

    // LiDAR mount offset: angle from the robot's +x (forward) to the
    // LiDAR's 0° scan direction, measured CCW. Added to every ray's CCW
    // angle before the cartesian conversion so pts_lidar ends up in the
    // robot's body frame — matching the convention used by the KF and
    // odometry (forward = +x, left = +y, θ CCW). Adjust this if pushing
    // the robot forward does *not* make Cox.X increase.
    //   0      : LiDAR's 0° already points forward
    //   -π/2   : LiDAR's 0° points to the robot's right (LiDAR rotated 90° CW)
    //   +π/2   : LiDAR's 0° points to the robot's left  (LiDAR rotated 90° CCW)
    //    π     : LiDAR's 0° points backward
    constexpr float LIDAR_MOUNT_OFFSET_RAD = -static_cast<float>(M_PI) / 2.0f;

    // Find the wall whose perpendicular distance to (pwx, pwy) is smallest.
    // Returns the wall index or -1 if no wall qualifies. y_signed is set to
    // the residual on success.
    int find_nearest_wall(const std::vector<LineSeg>& walls,
                    float pwx, float pwy, float& y_signed) {
        int   best_i   = -1;
        float best_abs = 1e30f;

        for (size_t i = 0; i < walls.size(); ++i) {
            const LineSeg& L = walls[i];
            const float dx = L.bx - L.ax;
            const float dy = L.by - L.ay;
            const float L2 = dx * dx + dy * dy;
            if (L2 <= 0.0f) continue;

            // t in [0, 1] is "on the segment". We allow ENDPOINT_SLACK_MM of
            // overshoot at each end so a near-corner ray isn't rejected.
            const float L_len   = std::sqrt(L2);
            const float t_param = ((pwx - L.ax) * dx + (pwy - L.ay) * dy) / L2;
            const float slack   = ENDPOINT_SLACK_MM / L_len;
            if (t_param < -slack || t_param > 1.0f + slack) continue;

            const float y  = L.nx * pwx + L.ny * pwy - L.r;
            const float ay = std::fabs(y);
            if (ay < best_abs) {
                best_abs = ay;
                best_i   = static_cast<int>(i);
                y_signed = y;
            }
        }
        return best_i;
    }

}  // namespace

CoxScanMatchAlgorithm::CoxScanMatchAlgorithm(const std::vector<LineSeg> &map,
                                            Pos2D initial_pos,
                                            LidarData& lidar_data_)
        : walls(map),
            current_pos(initial_pos),
            max_itr(30),
            lidar_data(lidar_data_),
            sigma_r(std::max(5.0f, static_cast<float>(lidar_data_.sensor_noise))),
            gate_k(4.0f),
            tol_d_xy(1.5f),
            tol_d_a(1.0e-3f) {
        pts_lidar.resize(2, 4096);
        pts_world.resize(2, 4096);
}


PosEstimate CoxScanMatchAlgorithm::estimate(Pos2D init_pos) {
    PosEstimate out = {
        current_pos,  // pos
        {1.0f, 0.0f, 0.0f,   // row 0: variance x, covariance xy, 0
        0.0f, 1.0f, 0.0f,   // row 1: covariance yx, variance y, 0
        0.0f, 0.0f, 0.0f},  // row 2: orientation terms (if 3x3)
        0, 0, 0,  // sigma values
        0, 0, false  // itr, inliers, converged
    };
    out.pos = init_pos;
    out.sigma_x = 0.0f;
    out.sigma_y = 0.0f;
    out.sigma_a = 0.0f;
    out.itr = 0;
    out.inliers = 0;
    out.converged = false;

    // 1. Snapshot the latest scan
    
    std::pair<std::vector<double>, std::vector<double>> ang_dist_pair = get_latest_frame_lidar_data(lidar_data);
    std::vector<double> angles = ang_dist_pair.first;
    std::vector<double> dist_s = ang_dist_pair.second;

    if (dist_s.size() < 8)
        return out;

    const int N = static_cast<int>(angles.size());

    // -- 2. Cartesian in lidar frame, computed once --------------------
    if (pts_lidar.cols() < N) {
        pts_lidar.resize(2, N);
        pts_world.resize(2, N);
    }
    for (int i = 0; i < N; ++i) {
        // LiDAR CW → math CCW, then rotate into the robot body frame so
        // forward = +x. See LIDAR_MOUNT_OFFSET_RAD above for the sign.
        const float a = -angles[i] * (M_PI / 180.0f) + LIDAR_MOUNT_OFFSET_RAD;
        pts_lidar(0, i) = dist_s[i] * std::cos(a);
        pts_lidar(1, i) = dist_s[i] * std::sin(a);
    }

    // --  3 iterations -----------------------------------------
    Pos2D pose = init_pos;
    Eigen::Matrix3f A;
    Eigen::Vector3f b;
    float ssq  = 0.0f;
    int   n_in = 0;

    const float final_gate = std::max(MIN_GATE_MM,
                                      std::min(MAX_GATE_MM, gate_k * sigma_r));
    // Start with a wide gate so a seeded pose that's tens of mm off still
    // captures correspondences on iter 0; shrink geometrically toward
    // final_gate so later iterations stay precise.
    const float start_gate = std::min(MAX_GATE_MM,
                                      std::max(final_gate, 50.0f));

    int  iters_done = 0;
    bool converged  = false;

    for (int iter = 0; iter < max_itr; ++iter) {
        const float gate = std::max(final_gate,
                                    start_gate * std::pow(0.6f,
                                                          static_cast<float>(iter)));
        const float c = std::cos(pose.theta);
        const float s = std::sin(pose.theta);

        // Rigid transform of all N points (Eigen vectorized).
        Eigen::Matrix2f R; R << c, -s, s, c;
        pts_world.leftCols(N).noalias() = R * pts_lidar.leftCols(N);
        pts_world.row(0).head(N).array() += pose.x;
        pts_world.row(1).head(N).array() += pose.y;

        A.setZero();
        b.setZero();
        ssq  = 0.0f;
        n_in = 0;

        for (int i = 0; i < N; ++i) {
            const float pwx = pts_world(0, i);
            const float pwy = pts_world(1, i);

            float y_i = 0.0f;
            const int w = find_nearest_wall(walls, pwx, pwy, y_i);
            if (w < 0) continue;
            if (std::fabs(y_i) > gate) continue;

            const float nx = walls[w].nx;
            const float ny = walls[w].ny;
            const float j3 = -nx * (pwy - pose.y) + ny * (pwx - pose.x);

            const Eigen::Vector3f Ji(nx, ny, j3);
            A.selfadjointView<Eigen::Lower>().rankUpdate(Ji);
            b.noalias() -= Ji * y_i;

            ssq += y_i * y_i;
            ++n_in;
        }
        A.triangularView<Eigen::StrictlyUpper>() = A.transpose();

        iters_done = iter + 1;
        if (n_in < 6) break;

        // Long corridor / single-
        // wall-orientation scans give a rank-deficient A whose LDLT either
        // fails or produces a near-singular inverse that blows the
        // covariance to Inf/NaN. A tiny diagonal ridge stabilizes both the
        // LDLT step solve below and the covariance computation outside the
        // loop without measurably biasing well-constrained matches (A's
        // diagonal entries grow with inlier count, typically in the
        // hundreds; the ridge is six orders of magnitude smaller).
        constexpr float RIDGE = 1e-3f;
        A(0, 0) += RIDGE;
        A(1, 1) += RIDGE;
        A(2, 2) += RIDGE;

        Eigen::LDLT<Eigen::Matrix3f> ldlt(A);
        if (ldlt.info() != Eigen::Success) break;

        const Eigen::Vector3f delta = ldlt.solve(b);
        pose.x += delta(0);
        pose.y += delta(1);
        pose.theta += delta(2);

        const float step_xy = std::hypot(delta(0), delta(1));
        if (step_xy < tol_d_xy && std::fabs(delta(2)) < tol_d_a) {
            converged = true;
            break;
        }
    }

    // Wrap theta into (-pi, pi].
    while (pose.theta >  M_PI) pose.theta -= 2.0f * M_PI;
    while (pose.theta <= -M_PI) pose.theta += 2.0f * M_PI;

    out.pos = pose;
    out.itr = iters_done;
    out.inliers = n_in;
    out.converged = converged && n_in >= 6;

    if (n_in >= 6) {
        // Even if we didn't formally converge, this pose is the best
        // information we have so we use it. Refusing to update on
        // non-convergence creates a feedback loop where the next solve
        // also diverges from a stale starting pose.
        const int   dof = std::max(1, n_in - 3);
        const float s2  = ssq / static_cast<float>(dof);
        const Eigen::Matrix3f cov = s2 * A.inverse();
        Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(out.cov) = cov;
        out.sigma_x = std::sqrt(std::max(0.0f, cov(0, 0)));
        out.sigma_y = std::sqrt(std::max(0.0f, cov(1, 1)));
        out.sigma_a = std::sqrt(std::max(0.0f, cov(2, 2)));
        current_pos = pose;
    }
    return out;
}

namespace {

// Trust gate for Cox measurements before they're fused into the EKF.
// Against the analytical 4-segment map, well-formed scans typically converge
// to sigma_xy ~ 5-15 mm and sigma_theta ~ 0.5-1.5 deg. Numbers above this
// gate indicate a bad initial seed, a shove in progress, or a degenerate
// scan (e.g. corridor view of just one wall) and should be rejected. Looser
// values were appropriate against the noisy RANSAC map -- with an exact
// map we can be much stricter, which also makes the kidnap-detection
// counter trigger faster.
constexpr int   MIN_INLIERS     = 10;
constexpr float MAX_SIGMA_XY    = 30.0f;   // mm  (~3 cm 1sigma)
constexpr float MAX_SIGMA_THETA = 0.035f;  // rad (~2 deg)

bool cox_trusted(const PosEstimate& est) {
    return est.converged
        && est.inliers >= MIN_INLIERS
        && est.sigma_x <  MAX_SIGMA_XY
        && est.sigma_y <  MAX_SIGMA_XY
        && est.sigma_a <  MAX_SIGMA_THETA;
}

// Coarse grid search across the arena. Iterate every (x, y, theta) seed and
// keep the Cox result with the most inliers (tie-break: smallest combined
// sigma) that still passes the standard trust gate. Returns true on success
// and writes the winning estimate to `out`. The arena bounding box comes
// from calib::get_arena_bounds() which was populated when the reference
// map was built.
bool relocalize_grid_sweep(CoxScanMatchAlgorithm& scan_cox,
                           double grid_mm, double grid_rad,
                           PosEstimate& out) {
    calib::ArenaBounds ab;
    if (!calib::get_arena_bounds(ab)) {
        LOGE("Cox") << "relocalize sweep: arena bounds not published";
        return false;
    }
    // Pull in slightly from the wall so seeds aren't on the boundary.
    const double margin = 100.0;
    const double x_min = ab.min_x + margin;
    const double x_max = ab.max_x - margin;
    const double y_min = ab.min_y + margin;
    const double y_max = ab.max_y - margin;
    if (x_max <= x_min || y_max <= y_min || grid_mm <= 0.0 || grid_rad <= 0.0) {
        LOGE("Cox") << "relocalize sweep: degenerate grid (bounds "
                    << x_min << ".." << x_max << ", " << y_min << ".." << y_max
                    << ", step " << grid_mm << " mm / " << grid_rad << " rad)";
        return false;
    }

    int    best_inliers = -1;
    float  best_sigma   = std::numeric_limits<float>::infinity();
    PosEstimate best{};
    int    tested = 0;

    for (double x = x_min; x <= x_max + 1e-6; x += grid_mm) {
        for (double y = y_min; y <= y_max + 1e-6; y += grid_mm) {
            for (double th = -M_PI; th < M_PI - 1e-6; th += grid_rad) {
                Pos2D seed(x, y, th);
                PosEstimate est = scan_cox.estimate(seed);
                ++tested;
                if (!cox_trusted(est)) continue;
                const float sig_sum = est.sigma_x + est.sigma_y;
                if (est.inliers > best_inliers ||
                    (est.inliers == best_inliers && sig_sum < best_sigma)) {
                    best_inliers = est.inliers;
                    best_sigma   = sig_sum;
                    best         = est;
                }
                if (app::shutdown.load()) return false;
            }
        }
    }
    if (best_inliers < 0) {
        LOGE("Cox") << "relocalize sweep: no trusted result across "
                    << tested << " seeds";
        return false;
    }
    LOG("Cox") << "relocalize sweep: winner at ("
               << best.pos.x << ", " << best.pos.y << ", "
               << (best.pos.theta * 180.0 / M_PI) << " deg) inliers="
               << best.inliers << " sigma=(" << best.sigma_x << ", "
               << best.sigma_y << ")  -- tested " << tested << " seeds";
    out = best;
    return true;
}

Pose2DCov moderate_recovery_cov() {
    Pose2DCov R;
    R.xx = 100.0 * 100.0;
    R.yy = 100.0 * 100.0;
    const double st = 5.0 * M_PI / 180.0;
    R.tt = st * st;
    return R;
}

}  // namespace

void loop_scan_cox_estimation(
    std::vector<LineSeg>& ref_map, LidarData& lidar_data,
    Dashboard& dash, KalmanFilter& kf,
    int kidnap_threshold,
    double relocalize_grid_mm,
    double relocalize_grid_rad) {
    Pos2D current_pos {
        0.0,0.0,0.0
    };
    CoxScanMatchAlgorithm scan_cox(ref_map, current_pos, lidar_data);
    // can't do empty init in C++11
    PosEstimate est = {
        current_pos,  // pos
        {1.0f, 0.0f, 0.0f,   // row 0: variance x, covariance xy, 0
        0.0f, 1.0f, 0.0f,   // row 1: covariance yx, variance y, 0
        0.0f, 0.0f, 0.0f},  // row 2: orientation terms (if 3x3)
        0, 0, 0,  // sigma values
        0, 0, false  // itr, inliers, converged
    };
    int consecutive_rejects = 0;
    while (!app::shutdown && !app::lidar_stop)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const Pos2D kf_pose = kf.pose();
        Pos2D seed;
        seed.x     = static_cast<float>(kf_pose.x);
        seed.y     = static_cast<float>(kf_pose.y);
        seed.theta = static_cast<float>(kf_pose.theta);
        current_pos = seed;
        est = scan_cox.estimate(current_pos);

        const bool trusted = cox_trusted(est);
        if (trusted) {
            // Fuse the scan-match into the KF as a full-pose measurement.
            Pos2D z;
            z.x     = est.pos.x;
            z.y     = est.pos.y;
            z.theta = est.pos.theta;
            Pose2DCov R;
            R.xx = est.cov[0]; R.xy = est.cov[1]; R.xt = est.cov[2];
            R.yy = est.cov[4]; R.yt = est.cov[5];
            R.tt = est.cov[8];
            kf.correct(z, R);
            current_pos = est.pos;
            consecutive_rejects = 0;
        } else {
            ++consecutive_rejects;
        }

        if (kidnap_threshold > 0 && consecutive_rejects >= kidnap_threshold) {
            LOGE("Cox") << "Kidnap suspected: " << consecutive_rejects
                        << " consecutive rejected scans -- starting "
                        << "relocalize sweep.";
            app::relocalizing.store(true);
            PosEstimate winner{};
            const bool found = relocalize_grid_sweep(
                scan_cox, relocalize_grid_mm, relocalize_grid_rad, winner);
            if (found) {
                kf.reset(winner.pos, moderate_recovery_cov());
                current_pos = winner.pos;
                est = winner;
                consecutive_rejects = 0;
                app::relocalizing.store(false);
                LOG("Cox") << "Relocalize OK -- KF re-pinned, resuming.";
            } else {
                app::relocalizing.store(false);
                LOGE("Cox") << "Relocalize failed -- robot is lost. "
                            << "Shutting down.";
                app::shutdown.store(true);
                break;
            }
        }

        // using both trusted and untrusted to logger
        // logger.on_cox_estimate(est, trusted);
        // Publish scan-match result to the dashboard. Published every cycle
        // (trusted or not) so a stale value never appears to be current; the
        // "status" column reflects whether the pose was accepted.
        dash.set("Cox", "X_cm", est.pos.x/10);
        dash.set("Cox", "Y_cm", est.pos.y/10);
        dash.set("Cox", "Th_deg", est.pos.theta * 180.0f / static_cast<float>(M_PI));
        dash.set("Cox", "sX_mm", est.sigma_x);
        dash.set("Cox", "sY_mm", est.sigma_y);
        dash.set("Cox", "sTh_deg", est.sigma_a * 180.0f / static_cast<float>(M_PI));
        dash.set("Cox", "inliers", static_cast<int64_t>(est.inliers));
        dash.set("Cox", "iters",   static_cast<int64_t>(est.itr));
        dash.set("Cox", "status",
                 std::string(trusted ? "OK" : (est.converged ? "weak" : "no-conv")));
        dash.set("Cox", "rejects", static_cast<int64_t>(consecutive_rejects));
    }
}
