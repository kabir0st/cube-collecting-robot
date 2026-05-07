#pragma once
#include <cmath>
#include <cstdint>
#include <vector>

#include <types.hpp>

// 2D point in millimetres, lidar/world frame.
struct Point {
    double x, y;
    Point(double x_ = 0.0, double y_ = 0.0) : x(x_), y(y_) {}
};

// Tunable parameters for fit_lines_ransac. Defaults work for a typical
// indoor box at ~25 mm sensor noise — calibration overrides
// inlier_thresh_mm with 3*sigma at runtime.
struct RansacParams {
    double   inlier_thresh_mm    = 25.0;     // perpendicular gate
    int      max_iters_per_line  = 200;      // inner RANSAC trials
    int      min_inliers         = 20;       // absolute floor per line
    double   min_inlier_fraction = 0.05;     // fraction of remaining cloud
    int      max_lines           = 16;       // hard cap on walls extracted
    double   endpoint_padding_mm = 50.0;     // extend segments past min/max
                                             // inlier projection by this much
                                             // so endpoint slack covers gaps
    uint32_t seed                = 0xC0FFEEu; // 0 -> std::random_device
};

// Iteratively extract line segments from a 2D point cloud:
//   1. RANSAC: sample two points, count inliers within `inlier_thresh_mm`
//      perpendicular distance, keep the best inlier set.
//   2. Refit on inliers via total least squares (orthogonal regression).
//   3. Recompute inlier set against the refit, take min/max projection
//      as endpoints, pad by `endpoint_padding_mm`.
//   4. Remove inliers, repeat until `min_inliers` or `min_inlier_fraction`
//      can no longer be met, or `max_lines` reached.
//
// Returns the segments in extraction order (longest/largest-support first).
std::vector<LineSeg> fit_lines_ransac(const std::vector<Point>& points,
                                      const RansacParams& p = {});
