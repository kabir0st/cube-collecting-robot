#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <types.hpp>

struct Point {
    double x, y;

    Point(double x_ = 0, double y_ = 0);
    double distance_to(const Point& other) const;
};

struct Rectangle {
    Point corners[4];   // CCW order in rotated frame: BL, BR, TR, TL
    LineSeg walls[4];   // wall i runs from corners[i] to corners[(i+1)%4]
    double theta;       // rotation of the rectangle's local axes (rad)
};

// Fit an axis-aligned-after-rotation rectangle to 2D points.
// Robust to outliers; uses minimum-area bounding rectangle search +
// per-side median. Throws on too few points.
Rectangle fit_rectangle(const std::vector<Point>& points);

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
