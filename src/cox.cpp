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
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <dashboard.hpp>
#include <kalman.hpp>

namespace {

    const float ENDPOINT_SLACK_MM = 100.0f;     // absolute slack along segment
    const float MAX_GATE_MM = 100.0f;
    const float MIN_GATE_MM  = 12.0f;

    // Shared with calibrations.cpp::find_walls via include/global.hpp so
    // the reference map and the per-tick scans use the same body-frame
    // transform.
    constexpr float LIDAR_MOUNT_OFFSET_F =
        static_cast<float>(::LIDAR_MOUNT_OFFSET_RAD);

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
                                            LidarData& lidar_data_)
        : walls(map),
            max_itr(50),
            lidar_data(lidar_data_),
            sigma_r(std::max(5.0f, static_cast<float>(lidar_data_.sensor_noise.load()))),
            gate_k(4.0f),
            // Realistic tolerances: lidar noise is ~10-20 mm so demanding
            // < 1.5 mm step convergence just exhausts the iter budget.
            tol_d_xy(5.0f),
            tol_d_a(2.0e-3f) {
        pts_lidar.resize(2, 4096);
        pts_world.resize(2, 4096);
}


PosEstimate CoxScanMatchAlgorithm::estimate(Pos2D init_pos) {
    PosEstimate out{};
    out.pos = init_pos;

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
        // forward = +x. See LIDAR_MOUNT_OFFSET_RAD in global.hpp for the sign.
        const float a = -angles[i] * (M_PI / 180.0f) + LIDAR_MOUNT_OFFSET_F;
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
        // Slower decay (0.75) so the gate stays wide for the first few
        // iterations gives a cold seed (50-100 mm off) room to find the
        // right correspondences before tightening for accuracy.
        const float gate = std::max(final_gate,
                                    start_gate * std::pow(0.75f,
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

        // Information-matrix condition number. A is symmetric PSD; the
        // SelfAdjointEigenSolver gives sorted real eigenvalues. A long
        // corridor (one wall visible) produces an A whose smallest
        // eigenvalue is dominated by the ridge so the ratio explodes
        // and cox_can_fuse() refuses to mix that estimate into the KF.
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolve(A);
        if (esolve.info() == Eigen::Success) {
            const float lam_min = esolve.eigenvalues()(0);
            const float lam_max = esolve.eigenvalues()(2);
            if (lam_min > 0.0f && std::isfinite(lam_max)) {
                out.cond_number = lam_max / lam_min;
            }
        }
    }
    return out;
}

namespace {

// Two gates, used in different places:
//
//   cox_can_fuse(): loose sanity floor. If true, we hand the estimate to
//   kf.correct() and let the KF's R-weighted gain do the work -- a noisy
//   cox (large sigma) automatically produces a weak correction, so we
//   gain nothing by gatekeeping ourselves on `converged`. The estimate's
//   covariance grows with ssq, so a poor-fit pose enters the KF with a
//   large R and barely moves the state. We refuse on actual garbage: too
//   few inliers, or rank-deficient geometry (sigma explodes in the
//   unconstrained direction -- e.g. single-wall corridor, sigma_y >> arena).
//
//   cox_strong_fix(): strict gate, used only to accept a relocalize sweep
//   winner. Because we're about to kf.reset() to it (blowing away prior
//   state), we need high confidence — there `converged` IS required.
//
// History: previously gated on `cond_number <= 1e4`. Killed every fuse in
// the 2-wall L-corner geometry (cond naturally ~1e6 even with sigma~2 mm)
// while letting through single-wall fits where one sigma is healthy and the
// other is 70 mm. Sigma gates directly check what cond was trying to proxy:
// "is the pose actually pinned in every axis?".
constexpr int   MIN_INLIERS_FUSE     = 20;
constexpr float MAX_SIGMA_XY_FUSE    = 30.0f;   // mm
constexpr float MAX_SIGMA_THETA_FUSE = 0.025f;  // rad (~1.4 deg)

// "Strong fix" thresholds — used only by relocalize_local_sweep() to accept
// a sweep winner before kf.reset(). Tightened for the 2.4m × 3.6m four-wall
// arena where well-observed scans routinely deliver sigma_xy ≈ 10-20 mm and
// sigma_theta ≈ 0.3-0.8°; the looser defaults (30 mm / 2°) accepted
// borderline fits that produced ~50 mm hops in the KF after relocalize.
constexpr int   MIN_INLIERS_FIX      = 20;
constexpr float MAX_SIGMA_XY_FIX     = 25.0f;   // mm  (~2.5 cm 1sigma)
constexpr float MAX_SIGMA_THETA_FIX  = 0.025f;  // rad (~1.4 deg)

bool cox_can_fuse(const PosEstimate& est) {
    // Note: `converged` is intentionally NOT in this gate. See block
    // comment above — a not-quite-converged estimate with healthy inliers
    // and finite sigmas is still useful information for the KF.
    return est.inliers >= MIN_INLIERS_FUSE
        && std::isfinite(est.sigma_x) && est.sigma_x < MAX_SIGMA_XY_FUSE
        && std::isfinite(est.sigma_y) && est.sigma_y < MAX_SIGMA_XY_FUSE
        && std::isfinite(est.sigma_a) && est.sigma_a < MAX_SIGMA_THETA_FUSE;
}

bool cox_strong_fix(const PosEstimate& est) {
    return est.converged
        && est.inliers >= MIN_INLIERS_FIX
        && std::isfinite(est.sigma_x) && est.sigma_x < MAX_SIGMA_XY_FIX
        && std::isfinite(est.sigma_y) && est.sigma_y < MAX_SIGMA_XY_FIX
        && std::isfinite(est.sigma_a) && est.sigma_a < MAX_SIGMA_THETA_FIX;
}

// LOCAL relocalize sweep. The professor only nudges the robot a bit, so
// the true pose stays within a small window around the current KF pose.
// We sweep a ±HALF_XY_MM by ±HALF_THETA_RAD box centered on the KF pose,
// run cox from each seed, and keep the strongest fix (most inliers,
// tie-break = smallest combined sigma). Returns false if no seed yields a
// strong fix; caller should keep predicting and try again next cycle.
bool relocalize_local_sweep(CoxScanMatchAlgorithm& scan_cox,
                            const Pos2D& center,
                            PosEstimate& out) {
    // Window large enough to absorb a meaningful shove but small enough
    // to finish in ~1-2 s. Grid step picked to fit inside cox's basin of
    // convergence on the arena map (~100 mm / ~15 deg).
    constexpr double HALF_XY_MM    = 400.0;
    constexpr double GRID_MM       = 100.0;
    constexpr double HALF_THETA    = M_PI / 4.0;        // ±45°
    constexpr double GRID_RAD      = 15.0 * M_PI / 180.0;

    int    best_inliers = -1;
    float  best_sigma   = std::numeric_limits<float>::infinity();
    PosEstimate best{};
    int    tested = 0;

    for (double dx = -HALF_XY_MM; dx <= HALF_XY_MM + 1e-6; dx += GRID_MM) {
        for (double dy = -HALF_XY_MM; dy <= HALF_XY_MM + 1e-6; dy += GRID_MM) {
            for (double dth = -HALF_THETA; dth <= HALF_THETA + 1e-6; dth += GRID_RAD) {
                Pos2D seed(center.x + dx, center.y + dy, center.theta + dth);
                PosEstimate est = scan_cox.estimate(seed);
                ++tested;
                if (!cox_strong_fix(est)) continue;
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
        LOGE("Cox") << "relocalize local sweep: no strong fix in "
                    << tested << " seeds around ("
                    << center.x << ", " << center.y << ", "
                    << (center.theta * 180.0 / M_PI) << " deg)";
        return false;
    }
    LOG("Cox") << "relocalize local sweep: winner at ("
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
    Dashboard& dash, KalmanFilter& kf, DataLogger& logger) {
    // KF-covariance thresholds for "I'm genuinely lost". This is the
    // principled signal: cumulative odometry drift + missing or weak cox
    // corrections push the KF's own confidence past safety. Stricter than
    // these and a brief cox dropout false-triggers a sweep; looser and
    // the robot drives blind for too long before recovering.
    //
    // Tuned for the 2.4m × 3.6m arena. The prior 300mm / 15° defaults are
    // generic for mid-sized halls: in this geometry sigma_xy = 300mm puts
    // the robot a foot past the closest wall before relocalize fires
    // (12% of the long axis), and 15° of heading uncertainty maps to
    // sin(15°) × 3.6m ≈ 930mm of lateral error at the far wall — way
    // beyond the home zone. Tightened to ~5.5% of long axis and 10° so
    // the sweep fires while there's still arena around the robot.
    constexpr double KF_LOST_SIGMA_XY_MM = 200.0;
    constexpr double KF_LOST_SIGMA_TH    = 10.0 * M_PI / 180.0;

    CoxScanMatchAlgorithm scan_cox(ref_map, lidar_data);
    PosEstimate est{};

    // Boot-sweep one-shot. If spontaneous per-tick fits don't produce a
    // first fix within BOOT_SWEEP_AFTER, run the same local sweep used
    // by the in-mission lost-recovery path around the seeded KF pose.
    // Catches the common failure mode where the map was built from a
    // pre-twitch pose but the body now sits tens of mm / a few degrees
    // off -- outside cox's per-tick basin (~50-100mm/15°) but well
    // within the sweep window (±400 mm / ±45°).
    const auto cox_loop_start = std::chrono::steady_clock::now();
    constexpr auto BOOT_SWEEP_AFTER = std::chrono::seconds(3);
    bool boot_sweep_done = false;

    while (!app::shutdown && !app::lidar_stop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Seed cox from the current KF pose.
        const Pos2D kf_pose = kf.pose();
        Pos2D seed;
        seed.x     = static_cast<float>(kf_pose.x);
        seed.y     = static_cast<float>(kf_pose.y);
        seed.theta = static_cast<float>(kf_pose.theta);
        est = scan_cox.estimate(seed);
        // Lab-3: log every cox attempt (converged or not). The native
        // loop sleep above is already 100 ms.
        logger.log_cox(logger.now_s(), est);

        // Push detection. When loop_control commanded a halt (v ≈ 0,
        // ω ≈ 0) yet a strong Cox fix sits materially far from the KF
        // mean, the chassis was likely moved by hand or slid on the
        // floor — the wheel-encoder prior is tight but wrong. Inflate
        // the KF position covariance once so the kf.correct() call
        // below has room to pull the prior to the new Cox estimate
        // instead of being dragged ~10 % per tick by the over-tight
        // prior. Gated on cox_strong_fix (not just can_fuse) so a
        // noisy single-wall fit can't yank the pose on its own; gated
        // on current_sigma_xy below the inflation level so we don't
        // re-inflate forever once it's already opened up.
        constexpr double PUSH_CMD_V_IDLE_MM_S   = 5.0;
        constexpr double PUSH_CMD_W_IDLE_RAD_S  = 0.05;
        constexpr double PUSH_DETECT_DIST_MM    = 60.0;
        constexpr double PUSH_INFLATE_SIGMA_MM  = 150.0;
        constexpr double PUSH_INFLATE_SIGMA_TH  = 8.0 * M_PI / 180.0;
        const double v_recent = std::fabs(app::v_cmd_last_mm_s.load());
        const double w_recent = std::fabs(app::w_cmd_last_rad_s.load());
        const bool commanded_idle =
            v_recent < PUSH_CMD_V_IDLE_MM_S
         && w_recent < PUSH_CMD_W_IDLE_RAD_S;
        if (commanded_idle && cox_strong_fix(est)) {
            const double dx_co = est.pos.x - kf_pose.x;
            const double dy_co = est.pos.y - kf_pose.y;
            const double dist_co = std::hypot(dx_co, dy_co);
            Pose2DCov cov_now = kf.covariance();
            const double sig_xy_now =
                std::sqrt(std::max(cov_now.xx, cov_now.yy));
            if (dist_co > PUSH_DETECT_DIST_MM
                && sig_xy_now < PUSH_INFLATE_SIGMA_MM) {
                LOG("Cox") << "Push suspected: commanded idle but Cox at ("
                           << est.pos.x << ", " << est.pos.y << ") is "
                           << dist_co << " mm from KF (" << kf_pose.x
                           << ", " << kf_pose.y << "); inflating prior.";
                kf.inflate(PUSH_INFLATE_SIGMA_MM * PUSH_INFLATE_SIGMA_MM,
                           PUSH_INFLATE_SIGMA_TH * PUSH_INFLATE_SIGMA_TH);
            }
        }

        // 1. ALWAYS-FUSE policy. If cox even minimally converged with a
        //    handful of inliers, hand it to the KF. A noisy cox produces a
        //    large R, the KF gain shrinks, the correction is weak
        const bool fuse = cox_can_fuse(est);
        if (fuse) {
            Pos2D z;
            z.x     = est.pos.x;
            z.y     = est.pos.y;
            z.theta = est.pos.theta;
            Pose2DCov R;
            R.xx = est.cov[0]; R.xy = est.cov[1]; R.xt = est.cov[2];
            R.yy = est.cov[4]; R.yt = est.cov[5];
            R.tt = est.cov[8];
            kf.correct(z, R);
            app::cox_first_fix.store(true, std::memory_order_release);
        }

        // 1b. BOOT SWEEP fallback. Runs at most once, only if spontaneous
        //     fits never converged in the opening window. relocalizing is
        //     set so loop_control holds the motors -- normally redundant
        //     during the first-fix wait, but defensive in case future
        //     mission logic ever races this path.
        if (!boot_sweep_done
            && !app::cox_first_fix.load(std::memory_order_acquire)
            && std::chrono::steady_clock::now() - cox_loop_start > BOOT_SWEEP_AFTER) {
            boot_sweep_done = true;
            LOG("Cox") << "no first fix after "
                       << BOOT_SWEEP_AFTER.count()
                       << "s -- running boot sweep around ("
                       << kf_pose.x << ", " << kf_pose.y << ", "
                       << (kf_pose.theta * 180.0 / M_PI) << " deg).";
            app::relocalizing.store(true);
            PosEstimate winner{};
            if (relocalize_local_sweep(scan_cox, kf_pose, winner)) {
                kf.reset(winner.pos, moderate_recovery_cov());
                est = winner;
                app::cox_first_fix.store(true, std::memory_order_release);
                LOG("Cox") << "boot sweep OK -- KF pinned to ("
                           << winner.pos.x << ", " << winner.pos.y << ", "
                           << (winner.pos.theta * 180.0 / M_PI) << " deg).";
            } else {
                LOGE("Cox") << "boot sweep produced no strong fix; "
                               "control will fall back to dead reckoning "
                               "when its wait expires.";
            }
            app::relocalizing.store(false);
        }

        // 2. KF-COV-TRIGGERED RELOCALIZE. After the correct above, check
        //    whether the KF's *own* covariance still says we're lost. A
        //    single good cox fix above will have pulled cov back down and
        //    short-circuited this path; only sustained dead reckoning +
        //    bad/missing cox lets us reach here.
        Pos2D     kf_now;
        Pose2DCov cov;
        kf.snapshot(kf_now, cov);
        const double sigma_xy = std::sqrt(std::max(cov.xx, cov.yy));
        const double sigma_th = std::sqrt(std::max(0.0, cov.tt));
        const bool   kf_lost  = sigma_xy > KF_LOST_SIGMA_XY_MM
                             || sigma_th > KF_LOST_SIGMA_TH;

        if (kf_lost) {
            LOGE("Cox") << "KF lost (sigma_xy=" << sigma_xy
                        << " mm, sigma_th=" << (sigma_th * 180.0 / M_PI)
                        << " deg) -- halting motors, running local sweep "
                        << "around (" << kf_now.x << ", " << kf_now.y << ").";
            app::relocalizing.store(true);
            PosEstimate winner{};
            if (relocalize_local_sweep(scan_cox, kf_now, winner)) {
                kf.reset(winner.pos, moderate_recovery_cov());
                est = winner;
                LOG("Cox") << "Relocalize OK -- KF re-pinned, resuming.";
                app::relocalizing.store(false);
            } else {
                // Stay halted; next iteration tries another sweep with a
                // fresh scan. Don't reset `relocalizing` -- motors stay
                // down until we have a fix.
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }

        // Dashboard. Always publish so a stale value never looks live.
        dash.set("Cox", "X_mm",     est.pos.x );
        dash.set("Cox", "Y_mm",     est.pos.y );
        dash.set("Cox", "Th_deg",   est.pos.theta * 180.0f / static_cast<float>(M_PI));
        dash.set("Cox", "sX_mm",    est.sigma_x);
        dash.set("Cox", "sY_mm",    est.sigma_y);
        dash.set("Cox", "sTh_deg",  est.sigma_a * 180.0f / static_cast<float>(M_PI));
        dash.set("Cox", "inliers",  static_cast<int64_t>(est.inliers));
        dash.set("Cox", "iters",    static_cast<int64_t>(est.itr));
        dash.set("Cox", "cond",
                 std::isfinite(est.cond_number) ? est.cond_number : -1.0f);
        dash.set("Cox", "status",
                 std::string(fuse ? "fused"
                                  : (est.converged ? "weak" : "no-conv")));
        dash.set("Cox", "kf_lost",  std::string(kf_lost ? "yes" : "no"));
    }
}
