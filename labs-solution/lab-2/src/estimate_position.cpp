#include <estimate_position.hpp>
#include <server.hpp>
#include <utils.hpp>
#include <global.hpp>
#include <log.hpp>

#include <Eigen/Cholesky>
#include <Eigen/LU>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>


// =============================================================================
// Cox '91 line-based scan matching
// =============================================================================
//
// For each scan point in the lidar frame (x_s, y_s), the world position
// given a candidate pose (tx, ty, theta) is:
//
//     p_w = R(theta) * (x_s, y_s) + (tx, ty)
//
// The signed perpendicular residual to a wall (n, r) is:
//
//     y_i = n . p_w - r
//
// The Jacobian of y_i wrt the pose increment (dx, dy, dtheta) is:
//
//     J_i = [ n_x, n_y, -n_x*(p_w.y - ty) + n_y*(p_w.x - tx) ]
//
// Stack all rows that survive the residual gate, solve the 3x3 normal
// equations (J^T J) * delta = -J^T y, update the pose by delta, repeat.
// Covariance ~ sigma^2 * (J^T J)^{-1}.


namespace {

const float PI_F             = 3.14159265358979323846f;
const float ENDPOINT_SLACK_MM = 50.0f;     // absolute slack along segment
const float MAX_GATE_MM       = 150.0f;
const float MIN_GATE_MM       = 10.0f;


// Find the wall whose perpendicular distance to (pwx, pwy) is smallest.
// Returns the wall index or -1 if no wall qualifies. y_signed is set to
// the residual on success.
int nearest_wall(const std::vector<LineSeg>& walls,
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


PositionEstimator::PositionEstimator(const std::vector<LineSeg>& ref_map,
                                     Pose2D initial_pose,
                                     int    frames_window,
                                     float  sensor_noise_mm,
                                     Loging* logger)
    : walls_(ref_map),
      current_pose_(initial_pose),
      frames_window_(frames_window > 0 ? frames_window : 1),
      max_iters_(15),
      sigma_r_(sensor_noise_mm > 1.0f ? sensor_noise_mm : 30.0f),
      gate_k_(3.0f),
      tol_dxy_(0.5f),
      tol_dtheta_(1e-4f),
      last_angle_(-1),
      logger_(logger)
{
    // ~22 lidar revolutions worth of points; resize() in estimate() is a
    // no-op until a scan exceeds this.
    P_lidar_.resize(2, 4096);
    P_world_.resize(2, 4096);
}


// Producer side: split the incoming stream into LidarDataFrame chunks
// at each ~360 -> ~0 wraparound. Keep `frames_window_` complete frames
// in the rolling buffer so estimate() always has fresh data.
void PositionEstimator::process_point(int quality, int angle, int distance) {
    if (quality <= 0 || distance <= 0) return;

    {
        std::lock_guard<std::mutex> lk(buf_mx_);

        // Frame boundary: angle dropped by > 90 deg => new revolution.
        if (last_angle_ >= 0 && angle + 90 < last_angle_ &&
            !in_progress_.angle.empty()) {
            frames_.push_back(std::move(in_progress_));
            in_progress_ = LidarDataFrame();
            while (static_cast<int>(frames_.size()) > frames_window_)
                frames_.pop_front();
        }

        in_progress_.angle.push_back(angle);
        in_progress_.distance.push_back(distance);
        last_angle_ = angle;
    }

    if (logger_) logger_->collect(angle, distance);
}


PoseEstimate PositionEstimator::estimate() {
    PoseEstimate out{};
    out.pose = current_pose_;

    // -- 1. Snapshot the buffered scan ---------------------------------
    std::vector<int> angles, distances;
    {
        std::lock_guard<std::mutex> lk(buf_mx_);
        size_t total = 0;
        for (const auto& f : frames_) total += f.angle.size();
        angles.reserve(total);
        distances.reserve(total);
        for (const auto& f : frames_) {
            angles.insert(angles.end(),    f.angle.begin(),    f.angle.end());
            distances.insert(distances.end(), f.distance.begin(), f.distance.end());
        }
    }
    if (angles.size() < 8) return out;
    const int N = static_cast<int>(angles.size());

    // -- 2. Cartesian in lidar frame, computed once --------------------
    if (P_lidar_.cols() < N) {
        P_lidar_.resize(2, N);
        P_world_.resize(2, N);
    }
    for (int i = 0; i < N; ++i) {
        const float a = angles[i] * (PI_F / 180.0f);
        P_lidar_(0, i) = distances[i] * std::cos(a);
        P_lidar_(1, i) = distances[i] * std::sin(a);
    }

    // -- 3. Newton iterations -----------------------------------------
    Pose2D pose = current_pose_;
    Eigen::Matrix3f A;
    Eigen::Vector3f b;
    float ssq  = 0.0f;
    int   n_in = 0;

    float gate = std::max(MIN_GATE_MM, std::min(MAX_GATE_MM, gate_k_ * sigma_r_));

    int  iters_done = 0;
    bool converged  = false;

    for (int iter = 0; iter < max_iters_; ++iter) {
        const float c = std::cos(pose.theta);
        const float s = std::sin(pose.theta);

        // Rigid transform of all N points (Eigen vectorized).
        Eigen::Matrix2f R; R << c, -s, s, c;
        P_world_.leftCols(N).noalias() = R * P_lidar_.leftCols(N);
        P_world_.row(0).head(N).array() += pose.x;
        P_world_.row(1).head(N).array() += pose.y;

        A.setZero();
        b.setZero();
        ssq  = 0.0f;
        n_in = 0;

        for (int i = 0; i < N; ++i) {
            const float pwx = P_world_(0, i);
            const float pwy = P_world_(1, i);

            float y_i = 0.0f;
            const int w = nearest_wall(walls_, pwx, pwy, y_i);
            if (w < 0) continue;
            if (std::fabs(y_i) > gate) continue;

            const float nx = walls_[w].nx;
            const float ny = walls_[w].ny;
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

        Eigen::LDLT<Eigen::Matrix3f> ldlt(A);
        if (ldlt.info() != Eigen::Success) break;

        const Eigen::Vector3f delta = ldlt.solve(b);
        pose.x     += delta(0);
        pose.y     += delta(1);
        pose.theta += delta(2);

        const float step_xy = std::hypot(delta(0), delta(1));
        if (step_xy < tol_dxy_ && std::fabs(delta(2)) < tol_dtheta_) {
            converged = true;
            break;
        }
    }

    // Wrap theta into (-pi, pi].
    while (pose.theta >  PI_F) pose.theta -= 2.0f * PI_F;
    while (pose.theta <= -PI_F) pose.theta += 2.0f * PI_F;

    out.pose       = pose;
    out.iterations = iters_done;
    out.inliers    = n_in;
    out.converged  = converged && n_in >= 6;

    if (n_in >= 6) {
        // Even if we didn't formally converge, this pose is the best
        // information we have — use it. Refusing to update on
        // non-convergence creates a feedback loop where the next solve
        // also diverges from a stale starting pose.
        const int   dof = std::max(1, n_in - 3);
        const float s2  = ssq / static_cast<float>(dof);
        const Eigen::Matrix3f cov = s2 * A.inverse();
        Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(out.cov) = cov;
        out.sigma_x     = std::sqrt(std::max(0.0f, cov(0, 0)));
        out.sigma_y     = std::sqrt(std::max(0.0f, cov(1, 1)));
        out.sigma_theta = std::sqrt(std::max(0.0f, cov(2, 2)));

        current_pose_ = pose;
    }
    return out;
}


// 1 Hz driver loop. Sets up a listener thread, starts the lidar, and
// prints one pose line per second until app::shutdown.
void do_estimate_position(const std::vector<LineSeg>& ref_map,
                          float sensor_noise_mm,
                          Loging* logger) {
    LOG("est") << "starting position estimation . . .";

    PositionEstimator estimator(ref_map,
                                Pose2D{0.0f, 0.0f, 0.0f},
                                /*frames_window=*/3,
                                sensor_noise_mm,
                                logger);

    ListenerThread listener(estimator, server);
    while (!server.is_ready() && !app::shutdown.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (app::shutdown.load()) {
        server.stop_stream();
        listener.join();
        return;
    }
    if (server.start_lidar() != 0) {
        server.stop_stream();
        listener.join();
        throw std::runtime_error("failed to start lidar for estimation");
    }

    // Warm-up: let frames_window_ revolutions land before the first solve.
    std::this_thread::sleep_for(std::chrono::milliseconds(800));

    auto next = std::chrono::steady_clock::now();
    while (!app::shutdown.load()) {
        next += std::chrono::seconds(1);

        const PoseEstimate est = estimator.estimate();
        LOG("pose") << "x="          << est.pose.x
                    << " y="         << est.pose.y
                    << " theta="     << (est.pose.theta * 180.0f / PI_F) << " deg"
                    << " | sigma=("  << est.sigma_x
                    << ", "          << est.sigma_y
                    << ", "          << (est.sigma_theta * 180.0f / PI_F) << " deg)"
                    << " inliers="   << est.inliers
                    << " iters="     << est.iterations
                    << (est.converged ? "  OK" : "  NOT-CONVERGED");

        std::this_thread::sleep_until(next);
    }

    // Order: unblock the recv loop, then join the listener so that
    // estimator.process_point cannot be called after estimator's
    // destructor begins running.
    server.stop_stream();
    listener.join();
}
