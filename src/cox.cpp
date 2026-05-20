#include <vector>
#include <cmath>
#include <map>
#include <cstring>

#include <cox.hpp>
#include <types.hpp>
#include <utils.hpp>
#include <global.hpp>
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
        // LiDAR CW → math CCW
        const float a = -angles[i] * (M_PI / 180.0f);
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

        // Tikhonov ridge on the Fisher-info matrix. Long corridor / single-
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

void loop_scan_cox_estimation(
    std::vector<LineSeg>& ref_map, LidarData& lidar_data,
    Dashboard& dash, KalmanFilter& kf) {
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

        constexpr int   MIN_INLIERS     = 10;
        constexpr float MAX_SIGMA_XY    = 100.0f;  // mm  (~10 cm 1sigma)
        constexpr float MAX_SIGMA_THETA = 0.087f;  // rad (~5 deg)
        const bool trusted = est.converged
                          && est.inliers >= MIN_INLIERS
                          && est.sigma_x <  MAX_SIGMA_XY
                          && est.sigma_y <  MAX_SIGMA_XY
                          && est.sigma_a <  MAX_SIGMA_THETA;
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
        }

        // using both trusted and untrusted to logger
        // logger.on_cox_estimate(est, trusted);
        // Publish scan-match result to the dashboard. Published every cycle
        // (trusted or not) so a stale value never appears to be current; the
        // "status" column reflects whether the pose was accepted.
        dash.set("Cox", "X", est.pos.x/10);
        dash.set("Cox", "Y", est.pos.y/10);
        dash.set("Cox", "theta_deg", est.pos.theta * 180.0f / static_cast<float>(M_PI));
        dash.set("Cox", "Sig_X", est.sigma_x);
        dash.set("Cox", "Sig_Y", est.sigma_y);
        dash.set("Cox", "sig_th_deg", est.sigma_a * 180.0f / static_cast<float>(M_PI));
        // dash.set("Cox Debug", "inliers", static_cast<int64_t>(est.inliers));
        // dash.set("Cox Debug", "iters", static_cast<int64_t>(est.itr));
        // dash.set("Cox Debug", "status", std::string(trusted ? "OK" : (est.converged ? "weak" : "no-conv")));
        // dash.set("Cox Debug", "time", get_timestamp());
    }
}