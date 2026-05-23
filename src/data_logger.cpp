#include <data_logger.hpp>

#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>

#include <loggers.hpp>

namespace {

std::string now_stamp() {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    // strftime instead of std::put_time — the Pi's libstdc++ is old enough
    // that put_time is missing from <iomanip> despite -std=c++11.
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
    return std::string(buf);
}

bool ensure_dir(const std::string& path) {
    if (path.empty()) return true;
    struct stat st{};
    if (stat(path.c_str(), &st) == 0) {
        return S_ISDIR(st.st_mode);
    }
    return mkdir(path.c_str(), 0755) == 0;
}

double safe_sigma(double var) {
    return std::sqrt(std::max(0.0, var));
}

}  // namespace

DataLogger::~DataLogger() {
    close();
}

bool DataLogger::open(bool enabled, const std::string& out_dir) {
    _enabled = false;
    _t0 = std::chrono::steady_clock::now();
    if (!enabled) {
        LOG("Logger") << "File logger disabled (logger.files_enabled=false).";
        return true;
    }

    if (!ensure_dir(out_dir)) {
        LOGE("Logger") << "Could not create out_dir '" << out_dir
                       << "' — file logger disabled.";
        return false;
    }

    const std::string stamp = now_stamp();
    const std::string base  = out_dir + "/" + stamp;

    _odom  .open(base + "_odom.csv",   std::ios::out | std::ios::trunc);
    _cox   .open(base + "_cox.csv",    std::ios::out | std::ios::trunc);
    _kalman.open(base + "_kalman.csv", std::ios::out | std::ios::trunc);

    if (!_odom || !_cox || !_kalman) {
        LOGE("Logger") << "Failed to open one or more CSV files under "
                       << out_dir << " — file logger disabled.";
        _odom.close(); _cox.close(); _kalman.close();
        return false;
    }

    _odom   << "t_s,x_mm,y_mm,theta_rad,sigma_x_mm,sigma_y_mm,sigma_theta_rad\n";
    _kalman << "t_s,x_mm,y_mm,theta_rad,sigma_x_mm,sigma_y_mm,sigma_theta_rad\n";
    _cox    << "t_s,x_mm,y_mm,theta_rad,sigma_x_mm,sigma_y_mm,sigma_theta_rad,"
               "converged,inliers,cond\n";

    _enabled = true;
    LOG("Logger") << "File logger active: " << base << "_{odom,cox,kalman}.csv";
    return true;
}

void DataLogger::close() {
    std::lock_guard<std::mutex> lk(_m);
    if (_odom)   _odom.flush();
    if (_cox)    _cox.flush();
    if (_kalman) _kalman.flush();
    _odom.close();
    _cox.close();
    _kalman.close();
    _enabled = false;
}

double DataLogger::now_s() const {
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now() - _t0).count();
}

void DataLogger::log_odom(double t_s, const Pos2D& p, const Pose2DCov& cov) {
    if (!_enabled) return;
    std::lock_guard<std::mutex> lk(_m);
    if (!_odom) return;
    _odom << t_s << ','
          << p.x << ',' << p.y << ',' << p.theta << ','
          << safe_sigma(cov.xx) << ',' << safe_sigma(cov.yy) << ','
          << safe_sigma(cov.tt) << '\n';
}

void DataLogger::log_kalman(double t_s, const Pos2D& p, const Pose2DCov& cov) {
    if (!_enabled) return;
    std::lock_guard<std::mutex> lk(_m);
    if (!_kalman) return;
    _kalman << t_s << ','
            << p.x << ',' << p.y << ',' << p.theta << ','
            << safe_sigma(cov.xx) << ',' << safe_sigma(cov.yy) << ','
            << safe_sigma(cov.tt) << '\n';
}

void DataLogger::log_cox(double t_s, const PosEstimate& est) {
    if (!_enabled) return;
    std::lock_guard<std::mutex> lk(_m);
    if (!_cox) return;
    _cox << t_s << ','
         << est.pos.x << ',' << est.pos.y << ',' << est.pos.theta << ','
         << est.sigma_x << ',' << est.sigma_y << ',' << est.sigma_a << ','
         << (est.converged ? 1 : 0) << ','
         << est.inliers << ','
         << est.cond_number << '\n';
}
