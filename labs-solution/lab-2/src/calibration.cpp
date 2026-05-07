#include <iostream>
#include <stdexcept>
#include <cmath>
#include <numeric>
#include <algorithm>

#include <calibrate.hpp>
#include <mapping.hpp>
#include <log.hpp>


// ---- Producer side --------------------------------------------------------
//
// Drop quality<=0 (driver flagged the return as bad) and distance<=0
// (out-of-range). For everything else we just bin by integer-degree
// angle; scan_collection_[a] is the list of distance samples seen at
// that bearing while the lidar was stationary.
void LidarCalibration::process_point(int quality, int angle, int distance) {
    if (quality <= 0 || distance <= 0) return;

    {
        std::lock_guard<std::mutex> g(mx_);
        scan_collection_[angle].push_back(distance);
    }
    if (logger_) logger_->collect(angle, distance);
}


// ---- Sigma estimation -----------------------------------------------------
//
// Each angle bin observes a different (unknown) true distance, so we
// pool the variance across bins instead of averaging means:
//
//      σ = sqrt( Σᵢ Σⱼ (dᵢⱼ - μᵢ)² / Σᵢ (nᵢ - 1) )
//
// Bins with fewer than 2 samples are excluded — they contribute zero
// degrees of freedom. Welford's one-pass form keeps the mean numerically
// stable even when distances are large and tightly clustered.
float LidarCalibration::calibrate() {
    std::lock_guard<std::mutex> g(mx_);

    if (scan_collection_.empty())
        throw std::runtime_error("calibrate: no scans collected");

    double sum_sq_dev = 0.0;
    long   total_dof  = 0;

    for (const auto& kv : scan_collection_) {
        const auto& samples = kv.second;
        if (samples.size() < 2) continue;

        double mean = 0.0, m2 = 0.0;
        size_t n = 0;
        for (int d : samples) {
            ++n;
            const double delta = d - mean;
            mean += delta / static_cast<double>(n);
            m2   += delta * (d - mean);
        }
        sum_sq_dev += m2;
        total_dof  += static_cast<long>(n) - 1;
    }

    lidar_sigma_ = (total_dof > 0)
        ? static_cast<float>(std::sqrt(sum_sq_dev / static_cast<double>(total_dof)))
        : 0.0f;
    return lidar_sigma_;
}


// ---- Reference map --------------------------------------------------------
//
// Average each bin to one (angle, distance) measurement, project to
// cartesian, then hand the cloud to RANSAC. The 3-sigma RANSAC
// threshold is floored at 15 mm so very-low-noise sensors don't
// fragment a single wall into multiple collinear pieces.
std::vector<LineSeg> LidarCalibration::create_reference_map() {
    std::vector<Point> pts;
    {
        std::lock_guard<std::mutex> g(mx_);
        pts.reserve(scan_collection_.size());
        for (const auto& kv : scan_collection_) {
            const auto& samples = kv.second;
            if (samples.empty()) continue;

            const double avg = std::accumulate(samples.begin(), samples.end(), 0.0)
                             / static_cast<double>(samples.size());
            if (avg < 150.0) continue;       // ignore very-near-zone noise

            const double a = kv.first * M_PI / 180.0;
            pts.emplace_back(avg * std::cos(a), avg * std::sin(a));
        }
    }
    LOG("map") << "data points: " << pts.size();

    RansacParams ransac;
    ransac.inlier_thresh_mm = std::max(15.0, 3.0 * static_cast<double>(lidar_sigma_));

    std::vector<LineSeg> walls = fit_lines_ransac(pts, ransac);

    LOG("map") << "inlier threshold: " << ransac.inlier_thresh_mm << " mm";
    LOG("map") << "walls extracted: " << walls.size();
    for (size_t i = 0; i < walls.size(); ++i) {
        const LineSeg& w = walls[i];
        LOG("map") << "  W" << i
                   << " n=(" << w.nx << ", " << w.ny << ") r=" << w.r
                   << "  (" << w.ax << ", " << w.ay
                   << ") -> (" << w.bx << ", " << w.by << ")";
    }

    if (walls.empty())
        throw std::runtime_error("create_reference_map: RANSAC found no walls");
    return walls;
}
