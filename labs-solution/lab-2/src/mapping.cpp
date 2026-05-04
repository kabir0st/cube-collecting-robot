#include "mapping.hpp"

// Point implementations
Point::Point(double x_, double y_) : x(x_), y(y_) {}

double Point::distance_to(const Point& other) const {
    double dx = x - other.x;
    double dy = y - other.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Line implementations
double Line::length() const {
    return start.distance_to(end);
}

double Line::distance_to(const Point& p) const {
    return std::abs(a * p.x + b * p.y + c) / std::sqrt(a * a + b * b);
}

double Line::angle() const {
    return std::atan2(end.y - start.y, end.x - start.x);
}
