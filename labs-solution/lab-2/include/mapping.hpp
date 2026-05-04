#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>

struct Point {
    double x, y;

    Point(double x_ = 0, double y_ = 0);
    double distance_to(const Point& other) const;
};

struct Line {
    Point start, end;
    // line: a*x + b*y + c = 0
    double a, b, c;
    // points belonging to this line
    std::vector<Point> points;

    double length() const;
    double distance_to(const Point& p) const;
    double angle() const;
};

class Map {

};
