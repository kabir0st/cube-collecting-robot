#include <cox_matcher.hpp>
#include <linmath.hpp>
#include <cmath>
#include <cstddef>

static const float PI_F = 3.14159265358979323846f;
static const float ENDPOINT_SLACK = 0.02f;
// Hard ceiling on the residual gate. Prevents a bad calibration sigma from
// silently disabling the gate and letting outliers bias the solve.
static const float MAX_GATE_MM = 150.0f;

CoxScanMatcher::CoxScanMatcher(std::vector<LineSeg> map, float sigma_r)
    : map_(std::move(map)),
      sigma_r_(sigma_r),
      max_iters_(10),
      gate_k_(3.0f),
      tol_dtheta_(1e-4f),
      tol_dxy_(0.5f) {}

int CoxScanMatcher::nearest_segment(float pwx, float pwy, float& y_signed) const {
    int best = -1;
    float best_abs = 1e30f;
    for (std::size_t i = 0; i < map_.size(); ++i) {
        const LineSeg& L = map_[i];
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
            best = static_cast<int>(i);
            y_signed = y;
        }
    }
    return best;
}

bool CoxScanMatcher::one_iteration(const LidarDataFrame& scan, Pose2D& pose,
                                   float A[9], float& s2, int& n_in) const {
    for (int i = 0; i < 9; ++i) A[i] = 0.0f;
    float b[3] = {0.0f, 0.0f, 0.0f};
    float ss = 0.0f;
    n_in = 0;

    const float c = std::cos(pose.theta);
    const float s = std::sin(pose.theta);
    float gate = gate_k_ * sigma_r_;
    if (gate > MAX_GATE_MM) gate = MAX_GATE_MM;
    if (gate < 10.0f) gate = 10.0f;

    const std::size_t n = scan.angle.size();
    for (std::size_t i = 0; i < n; ++i) {
        const int d = scan.distance[i];
        if (d <= 0) continue;
        const float a = scan.angle[i] * (PI_F / 180.0f);
        const float xs = d * std::cos(a);
        const float ys = d * std::sin(a);
        const float pwx = c * xs - s * ys + pose.x;
        const float pwy = s * xs + c * ys + pose.y;

        float y_i = 0.0f;
        const int seg = nearest_segment(pwx, pwy, y_i);
        if (seg < 0) continue;
        if (std::fabs(y_i) > gate) continue;

        const float nx = map_[seg].nx;
        const float ny = map_[seg].ny;
        // d(y_i)/d(theta) = nx*(-(pwy-ty)) + ny*(pwx-tx)
        const float j3 = -nx * (pwy - pose.y) + ny * (pwx - pose.x);

        // Accumulate A = J^T J, b = -J^T y_i (residual reduces as pose updates).
        A[0] += nx * nx;
        A[1] += nx * ny;
        A[2] += nx * j3;
        A[4] += ny * ny;
        A[5] += ny * j3;
        A[8] += j3 * j3;

        b[0] += -nx * y_i;
        b[1] += -ny * y_i;
        b[2] += -j3 * y_i;

        ss += y_i * y_i;
        ++n_in;
    }
    A[3] = A[1];
    A[6] = A[2];
    A[7] = A[5];

    if (n_in < 3) return false;

    float delta[3];
    if (!solve3x3(A, b, delta)) return false;

    pose.x     += delta[0];
    pose.y     += delta[1];
    pose.theta += delta[2];

    const int dof = (n_in - 3 > 0) ? (n_in - 3) : 1;
    s2 = ss / static_cast<float>(dof);

    // Convergence check on this step's delta is done by the caller.
    return std::fabs(delta[2]) < tol_dtheta_ &&
           std::sqrt(delta[0] * delta[0] + delta[1] * delta[1]) < tol_dxy_;
}

TrackerResult CoxScanMatcher::match(const LidarDataFrame& scan, const Pose2D& prior) {
    TrackerResult result;
    result.pose = prior;
    for (int i = 0; i < 9; ++i) result.cov[i] = 0.0f;
    result.iterations = 0;
    result.inliers = 0;
    result.converged = false;

    if (map_.size() < 2 || scan.angle.empty() || scan.angle.size() != scan.distance.size()) {
        return result;
    }

    Pose2D pose = prior;
    float A[9] = {0.0f};
    float s2 = 0.0f;
    int n_in = 0;
    bool converged = false;

    for (int iter = 0; iter < max_iters_; ++iter) {
        const bool small_step = one_iteration(scan, pose, A, s2, n_in);
        result.iterations = iter + 1;
        result.inliers = n_in;
        if (n_in < 3) {
            // singular or too few inliers: keep prior
            return result;
        }
        if (small_step) { converged = true; break; }
    }

    // Normalize theta to (-pi, pi]
    while (pose.theta >  PI_F) pose.theta -= 2.0f * PI_F;
    while (pose.theta <= -PI_F) pose.theta += 2.0f * PI_F;

    // Final A and s2 already populated by last one_iteration().
    float Ainv[9];
    if (!inverse3x3(A, Ainv)) return result;

    for (int i = 0; i < 9; ++i) result.cov[i] = s2 * Ainv[i];
    result.pose = pose;
    result.converged = converged;
    return result;
}
