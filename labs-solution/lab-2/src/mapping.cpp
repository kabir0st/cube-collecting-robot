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

bool Line::check_fit(const Point& p, float threshold){
    if (na * p.x + nb * p.y + nc > threshold){
        return false;
    }
    return true;
}

void Line::add_point(const Point &p) {
    points.push_back(p);
    end = p;
    a = end.y - start.y;
    b = end.x - start.x;
    c = start.x * end.y - end.x * start.y;
    double d = start.distance_to(end);
    na = a / d;
    nb = b / d;
    nc = c / d;
}

namespace {

double percentile_sorted(const std::vector<double>& sorted_vals, double q) {
    if (sorted_vals.empty()) return 0.0;
    double idx = q * (sorted_vals.size() - 1);
    size_t i = static_cast<size_t>(idx);
    if (i + 1 >= sorted_vals.size()) return sorted_vals.back();
    double frac = idx - static_cast<double>(i);
    return sorted_vals[i] * (1.0 - frac) + sorted_vals[i + 1] * frac;
}

double median_of(std::vector<double>& v) {
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

double bbox_area_at(const std::vector<Point>& pts, double theta) {
    const double c = std::cos(-theta);
    const double s = std::sin(-theta);
    std::vector<double> xs, ys;
    xs.reserve(pts.size());
    ys.reserve(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
        xs.push_back(c * pts[i].x - s * pts[i].y);
        ys.push_back(s * pts[i].x + c * pts[i].y);
    }
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());
    double w = percentile_sorted(xs, 0.98) - percentile_sorted(xs, 0.02);
    double h = percentile_sorted(ys, 0.98) - percentile_sorted(ys, 0.02);
    return w * h;
}

}  // namespace

Rectangle fit_rectangle(const std::vector<Point>& points) {
    if (points.size() < 16) {
        throw std::runtime_error("fit_rectangle: need at least 16 points");
    }

    double best_theta = 0.0;
    double best_area = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 90; ++i) {
        double t = i * M_PI / 180.0;
        double a = bbox_area_at(points, t);
        if (a < best_area) { best_area = a; best_theta = t; }
    }
    const double fine_step = 0.05 * M_PI / 180.0;
    for (int i = -20; i <= 20; ++i) {
        double t = best_theta + i * fine_step;
        double a = bbox_area_at(points, t);
        if (a < best_area) { best_area = a; best_theta = t; }
    }

    const double cN = std::cos(-best_theta);
    const double sN = std::sin(-best_theta);
    std::vector<Point> rot;
    rot.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        rot.push_back(Point(cN * points[i].x - sN * points[i].y,
                            sN * points[i].x + cN * points[i].y));
    }

    std::vector<double> xs, ys;
    xs.reserve(rot.size());
    ys.reserve(rot.size());
    for (size_t i = 0; i < rot.size(); ++i) {
        xs.push_back(rot[i].x);
        ys.push_back(rot[i].y);
    }
    std::sort(xs.begin(), xs.end());
    std::sort(ys.begin(), ys.end());
    const double cx = xs[xs.size() / 2];
    const double cy = ys[ys.size() / 2];

    std::vector<double> left_xs, right_xs, bot_ys, top_ys;
    for (size_t i = 0; i < rot.size(); ++i) {
        double dx = rot[i].x - cx;
        double dy = rot[i].y - cy;
        if (std::abs(dx) >= std::abs(dy)) {
            (dx < 0 ? left_xs : right_xs).push_back(rot[i].x);
        } else {
            (dy < 0 ? bot_ys : top_ys).push_back(rot[i].y);
        }
    }
    if (left_xs.size() < 3 || right_xs.size() < 3 ||
        bot_ys.size() < 3 || top_ys.size() < 3) {
        throw std::runtime_error("fit_rectangle: a side has too few points");
    }

    const double xL = median_of(left_xs);
    const double xR = median_of(right_xs);
    const double yB = median_of(bot_ys);
    const double yT = median_of(top_ys);

    Point r0(xL, yB), r1(xR, yB), r2(xR, yT), r3(xL, yT);

    const double cB = std::cos(best_theta);
    const double sB = std::sin(best_theta);
    Rectangle rect;
    rect.theta = best_theta;
    Point rcorners[4] = {r0, r1, r2, r3};
    for (int i = 0; i < 4; ++i) {
        rect.corners[i] = Point(cB * rcorners[i].x - sB * rcorners[i].y,
                                sB * rcorners[i].x + cB * rcorners[i].y);
    }

    for (int i = 0; i < 4; ++i) {
        const Point& A = rect.corners[i];
        const Point& B = rect.corners[(i + 1) % 4];
        LineSeg seg;
        seg.ax = static_cast<float>(A.x);
        seg.ay = static_cast<float>(A.y);
        seg.bx = static_cast<float>(B.x);
        seg.by = static_cast<float>(B.y);
        double dx = B.x - A.x;
        double dy = B.y - A.y;
        double len = std::sqrt(dx * dx + dy * dy);
        double nx = -dy / len;
        double ny = dx / len;
        double mx = 0.5 * (A.x + B.x);
        double my = 0.5 * (A.y + B.y);
        if (nx * mx + ny * my < 0) { nx = -nx; ny = -ny; }
        seg.nx = static_cast<float>(nx);
        seg.ny = static_cast<float>(ny);
        seg.r  = static_cast<float>(nx * A.x + ny * A.y);
        rect.walls[i] = seg;
    }

    return rect;
}