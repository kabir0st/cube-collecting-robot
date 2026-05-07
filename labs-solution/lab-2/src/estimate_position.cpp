#include <estimate_position.hpp>
#include <server.hpp>
#include <utils.hpp>
#include <global.hpp>

#include <cmath>
#include <chrono>
#include <thread>
#include <iostream>
#include <stdexcept>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/LU>


// Cox '91 line-based scan matching, simple form.
//
// Model:
//   For each scan point in the lidar frame (x_s, y_s), the world position
//   given a pose (tx, ty, theta) is:
//       p_w = R(theta) * (x_s, y_s) + (tx, ty)
//   The signed perpendicular residual to a wall (n, r) is:
//       y_i = n . p_w - r
//   The Jacobian of y_i wrt the pose increment (dx, dy, dtheta) is:
//       J_i = [ n_x,
//               n_y,
//               -n_x * (p_w.y - ty) + n_y * (p_w.x - tx) ]
//   Stack all N rows, solve the 3x3 normal equations:
//       (J^T J) * delta = -J^T y
//   Update pose by delta and iterate. Covariance ~ sigma^2 * (J^T J)^{-1}.


namespace {

const float PI_F = 3.14159265358979323846f;
const float ENDPOINT_SLACK = 0.02f;   // 2% tolerance off either end of a wall.
const float MAX_GATE_MM    = 150.0f;  // Hard ceiling on the residual gate.
const float MIN_GATE_MM    = 10.0f;


// Find the wall whose perpendicular distance to (pwx, pwy) is smallest,
// within endpoint slack. Returns -1 if no wall qualifies.
int nearest_wall(const LineSeg walls[4], float pwx, float pwy, float& y_signed) {
    int best = -1;
    float best_abs = 1e30f;
    for (int i = 0; i < 4; ++i) {
        const LineSeg& L = walls[i];
        const float dx = L.bx - L.ax;
        const float dy = L.by - L.ay;
        const float len2 = dx * dx + dy * dy;
        if (len2 <= 0.0f) continue;
        const float t = ((pwx - L.ax) * dx + (pwy - L.ay) * dy) / len2;
        if (t < -ENDPOINT_SLACK || t > 1.0f + ENDPOINT_SLACK) continue;
        const float y = L.nx * pwx + L.ny * pwy - L.r;
        const float ay = std::fabs(y);
        if (ay < best_abs) {
            best_abs = ay;
            best = i;
            y_signed = y;
        }
    }
    return best;
}

}  // namespace


PositionEstimator::PositionEstimator(const Rectangle& ref_map,
                                     Pose2D initial_pose,
                                     int frames_window,
                                     float sensor_noise_mm)
    : current_pose_(initial_pose),
      frames_window_(frames_window > 0 ? frames_window : 1),
      max_iters_(15),
      sigma_r_(sensor_noise_mm > 1.0f ? sensor_noise_mm : 30.0f),
      gate_k_(3.0f),
      tol_dxy_(0.5f),
      tol_dtheta_(1e-4f),
      last_angle_(-1) {
    for (int i = 0; i < 4; ++i) walls_[i] = ref_map.walls[i];
    // Reserve enough capacity for ~22 lidar revolutions; resize() inside
    // estimate() is then a no-op as long as N stays below this.
    P_lidar_.resize(2, 4096);
    P_world_.resize(2, 4096);
}


void PositionEstimator::process_point(int quality, int angle, int distance) {
    if (quality <= 0 || distance <= 0) return;

    std::lock_guard<std::mutex> lk(buf_mx_);
    // Frame boundary heuristic: angle drops by > 90 deg => new revolution.
    if (last_angle_ >= 0 && angle + 90 < last_angle_) {
        if (!in_progress_.angle.empty()) {
            in_progress_.num_points = static_cast<int>(in_progress_.angle.size());
            frames_.push_back(std::move(in_progress_));
            in_progress_ = LidarDataFrame();
            while (static_cast<int>(frames_.size()) > frames_window_) {
                frames_.pop_front();
            }
        }
    }
    in_progress_.angle.push_back(angle);
    in_progress_.distance.push_back(distance);
    last_angle_ = angle;
}


PoseEstimate PositionEstimator::estimate() {
    PoseEstimate out;
    out.pose = current_pose_;
    for (int i = 0; i < 9; ++i) out.cov[i] = 0.0f;
    out.sigma_x = out.sigma_y = out.sigma_theta = 0.0f;
    out.iterations = 0;
    out.inliers = 0;
    out.converged = false;

    // Snapshot the buffered scan points.
    std::vector<int> angles;
    std::vector<int> distances;
    {
        std::lock_guard<std::mutex> lk(buf_mx_);
        size_t total = 0;
        for (size_t i = 0; i < frames_.size(); ++i) total += frames_[i].angle.size();
        angles.reserve(total);
        distances.reserve(total);
        for (size_t i = 0; i < frames_.size(); ++i) {
            const std::vector<int>& a = frames_[i].angle;
            const std::vector<int>& d = frames_[i].distance;
            angles.insert(angles.end(), a.begin(), a.end());
            distances.insert(distances.end(), d.begin(), d.end());
        }
    }
    if (angles.size() < 8) return out;
    const int N = static_cast<int>(angles.size());

    // Precompute lidar-frame cartesian coords once into a 2xN Eigen matrix.
    // The per-iteration world transform is then a single Matrix2f * Matrix2Xf
    // multiply, which Eigen vectorizes (NEON on Cortex-A53).
    if (P_lidar_.cols() < N) {
        P_lidar_.resize(2, N);
        P_world_.resize(2, N);
    }
    for (int i = 0; i < N; ++i) {
        const float a = angles[i] * (PI_F / 180.0f);
        P_lidar_(0, i) = distances[i] * std::cos(a);
        P_lidar_(1, i) = distances[i] * std::sin(a);
    }

    Pose2D pose = current_pose_;
    Eigen::Matrix3f A = Eigen::Matrix3f::Zero();
    Eigen::Vector3f b = Eigen::Vector3f::Zero();
    float ssq  = 0.0f;
    int   n_in = 0;
    bool  converged = false;

    float gate = gate_k_ * sigma_r_;
    if (gate > MAX_GATE_MM) gate = MAX_GATE_MM;
    if (gate < MIN_GATE_MM) gate = MIN_GATE_MM;

    int iter = 0;
    for (; iter < max_iters_; ++iter) {
        const float c = std::cos(pose.theta);
        const float s = std::sin(pose.theta);

        Eigen::Matrix2f R;
        R << c, -s,
             s,  c;
        // Vectorized rigid transform: P_world = R * P_lidar + t.
        P_world_.leftCols(N).noalias() = R * P_lidar_.leftCols(N);
        P_world_.row(0).head(N).array() += pose.x;
        P_world_.row(1).head(N).array() += pose.y;

        A.setZero();
        b.setZero();
        ssq = 0.0f;
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
            // Symmetric rank-1 update of A = sum Ji * Ji^T (lower triangle).
            A.selfadjointView<Eigen::Lower>().rankUpdate(Ji);
            b.noalias() -= Ji * y_i;

            ssq += y_i * y_i;
            ++n_in;
        }
        // Mirror lower into upper so A is fully populated.
        A.triangularView<Eigen::StrictlyUpper>() = A.transpose();

        if (n_in < 6) break;

        Eigen::LDLT<Eigen::Matrix3f> ldlt(A);
        if (ldlt.info() != Eigen::Success) break;
        const Eigen::Vector3f delta = ldlt.solve(b);

        pose.x     += delta(0);
        pose.y     += delta(1);
        pose.theta += delta(2);

        const float step_xy = std::sqrt(delta(0) * delta(0) + delta(1) * delta(1));
        if (step_xy < tol_dxy_ && std::fabs(delta(2)) < tol_dtheta_) {
            converged = true;
            ++iter;
            break;
        }
    }

    while (pose.theta >  PI_F) pose.theta -= 2.0f * PI_F;
    while (pose.theta <= -PI_F) pose.theta += 2.0f * PI_F;

    out.pose = pose;
    out.iterations = iter;
    out.inliers = n_in;
    out.converged = converged && n_in >= 6;

    if (n_in >= 6) {
        const int dof = std::max(1, n_in - 3);
        const float s2 = ssq / static_cast<float>(dof);
        const Eigen::Matrix3f cov = s2 * A.inverse();
        Eigen::Map<Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(out.cov) = cov;
        out.sigma_x     = std::sqrt(std::max(0.0f, cov(0, 0)));
        out.sigma_y     = std::sqrt(std::max(0.0f, cov(1, 1)));
        out.sigma_theta = std::sqrt(std::max(0.0f, cov(2, 2)));
    }

    if (out.converged) current_pose_ = pose;
    return out;
}


void do_estimate_position(const Rectangle& ref_map, float sensor_noise_mm) {
    std::cout << "[estimate] Starting position estimation . . ." << std::endl;

    PositionEstimator estimator(ref_map, Pose2D{0.0f, 0.0f, 0.0f},
                                /*frames_window=*/3, sensor_noise_mm);

    ListenerThread thread_listener(estimator, server);
    while (!server.is_ready() && !app::shutdown.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (app::shutdown.load()) {
        server.stop_stream();
        return;
    }
    if (server.start_lidar() != 0) {
        server.stop_stream();
        throw std::runtime_error("failed to start lidar for estimation");
    }

    // Warm up: let the buffer fill before the first solve.
    std::this_thread::sleep_for(std::chrono::milliseconds(800));

    auto next = std::chrono::steady_clock::now();
    while (!app::shutdown.load()) {
        next += std::chrono::seconds(1);

        PoseEstimate est = estimator.estimate();
        std::cout << "[pose] x="     << est.pose.x
                  << " y="           << est.pose.y
                  << " theta="       << (est.pose.theta * 180.0f / PI_F) << " deg"
                  << " | sigma=("    << est.sigma_x
                  << ", "            << est.sigma_y
                  << ", "            << (est.sigma_theta * 180.0f / PI_F) << " deg)"
                  << " inliers="     << est.inliers
                  << " iters="       << est.iterations
                  << (est.converged ? "  OK" : "  NOT-CONVERGED")
                  << std::endl;

        std::this_thread::sleep_until(next);
    }

    server.stop_stream();
}
