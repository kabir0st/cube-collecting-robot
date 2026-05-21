#include <kalman.hpp>

#include <cmath>
#include <Eigen/Core>
#include <Eigen/LU>

#include <utils.hpp>

namespace {
    Eigen::Matrix3d to_eigen(const Pose2DCov& c) {
        Eigen::Matrix3d M;
        M(0,0) = c.xx; M(0,1) = c.xy; M(0,2) = c.xt;
        M(1,0) = c.xy; M(1,1) = c.yy; M(1,2) = c.yt;
        M(2,0) = c.xt; M(2,1) = c.yt; M(2,2) = c.tt;
        return M;
    }

    Pose2DCov from_eigen(const Eigen::Matrix3d& M) {
        Pose2DCov c;
        c.xx = M(0,0); c.xy = 0.5 * (M(0,1) + M(1,0));
        c.xt = 0.5 * (M(0,2) + M(2,0));
        c.yy = M(1,1); c.yt = 0.5 * (M(1,2) + M(2,1));
        c.tt = M(2,2);
        return c;
    }
}

Pose2DCov KalmanFilter::default_cov() {
    // Boot seed when we *expect* the robot is at home. If it isn't,
    // loop_scan_cox_estimation's first untrusted estimate triggers a
    // relocalize sweep which re-pins the KF with moderate_recovery_cov().
    Pose2DCov c;
    c.xx = 25.0 * 25.0; // sigma_x = 25 mm
    c.yy = 25.0 * 25.0; // sigma_y = 25 mm
    const double st = 2.0 * M_PI / 180.0; // sigma_theta = 2 deg
    c.tt = st * st;
    return c;
}

KalmanFilter::KalmanFilter() {
    _cov = default_cov();
}

void KalmanFilter::reset(const Pos2D& pose, const Pose2DCov& cov) {
    std::lock_guard<std::mutex> lk(_m);
    _pose = pose;
    _cov  = cov;
}

void KalmanFilter::predict(double d, double d_theta,
                           double var_d, double var_theta,
                           double cov_d_theta) {
    std::lock_guard<std::mutex> lk(_m);

    // Midpoint heading. Mean and Jacobians are linearized about theta_m.
    const double theta_m = _pose.theta + 0.5 * d_theta;
    const double c = std::cos(theta_m);
    const double s = std::sin(theta_m);
    const double h = 0.5 * d;

    // Mean update.
    _pose.x    += d * c;
    _pose.y    += d * s;
    _pose.theta = wrap_pi(_pose.theta + d_theta);

    // State Jacobian Fx.
    Eigen::Matrix3d Fx;
    Fx << 1.0, 0.0, -d * s,
          0.0, 1.0,  d * c,
          0.0, 0.0,  1.0;

    // Input Jacobian Fu w.r.t. (d, d_theta), evaluated at midpoint.
    // The (h * s, h * c) terms inject heading-position correlation into Q,
    // which is what saves the filter from heading-overconfidence later.
    Eigen::Matrix<double, 3, 2> Fu;
    Fu << c,     -h * s,
          s,      h * c,
          0.0,    1.0;

    Eigen::Matrix2d Q;
    Q(0, 0) = var_d;        Q(0, 1) = cov_d_theta;
    Q(1, 0) = cov_d_theta;  Q(1, 1) = var_theta;

    Eigen::Matrix3d P = to_eigen(_cov);
    P = Fx * P * Fx.transpose() + Fu * Q * Fu.transpose();
    _cov = from_eigen(P);
}

void KalmanFilter::correct(const Pos2D& z, const Pose2DCov& R_in) {
    std::lock_guard<std::mutex> lk(_m);

    Eigen::Vector3d y;
    y(0) = z.x     - _pose.x;
    y(1) = z.y     - _pose.y;
    y(2) = wrap_pi(z.theta - _pose.theta);

    const Eigen::Matrix3d P = to_eigen(_cov);
    const Eigen::Matrix3d R = to_eigen(R_in);
    const Eigen::Matrix3d S = P + R;
    const Eigen::Matrix3d K = P * S.inverse();

    // State update.
    const Eigen::Vector3d update = K * y;
    _pose.x    += update(0);
    _pose.y    += update(1);
    _pose.theta = wrap_pi(_pose.theta + update(2));

    // Joseph-form covariance update — symmetric and PSD-preserving even
    // under finite-precision K.
    const Eigen::Matrix3d I       = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d I_min_K = I - K;
    const Eigen::Matrix3d P_new   = I_min_K * P * I_min_K.transpose()
                                  + K * R * K.transpose();
    _cov = from_eigen(P_new);
}

Pos2D KalmanFilter::pose() const {
    std::lock_guard<std::mutex> lk(_m);
    return _pose;
}

Pose2DCov KalmanFilter::covariance() const {
    std::lock_guard<std::mutex> lk(_m);
    return _cov;
}

void KalmanFilter::snapshot(Pos2D& out_pose, Pose2DCov& out_cov) const {
    std::lock_guard<std::mutex> lk(_m);
    out_pose = _pose;
    out_cov  = _cov;
}
