#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <cstdint>
#include <types.hpp>

struct Point {
    double x, y;

    Point(double x_ = 0, double y_ = 0);
    double distance_to(const Point& other) const;
};

struct RansacParams {
    double   inlier_thresh_mm    = 25.0;     // perpendicular distance gate
    int      max_iters_per_line  = 200;      // inner RANSAC trials
    int      min_inliers         = 20;       // absolute floor
    double   min_inlier_fraction = 0.05;     // of remaining-point set
    int      max_lines           = 16;       // hard cap
    uint32_t seed                = 0xC0FFEEu; // 0 -> std::random_device
};

// Iteratively extract line segments from a 2D point cloud via RANSAC.
// Each iteration: sample two points, count inliers within
// `inlier_thresh_mm` perpendicular distance, refit on inliers via
// total least squares, derive endpoints from inlier projection,
// remove inliers, repeat. Stops when the remaining set is too small
// or no candidate line clears `min_inliers`.
std::vector<LineSeg> fit_lines_ransac(const std::vector<Point>& points,
                                      const RansacParams& p = {});

struct Line {
    public:
        Point start, end;
        double a, b, c;
        // normalized line;
        double na, nb, nc;
        // points belonging to this line
        std::vector<Point> points;

        Line(const Point& p) : start(p) {};
        double length() const;
        double distance_to(const Point& p) const;
        double angle() const;
        bool check_fit(const Point& p, float threshold);
        void add_point(const Point &p);
    };
