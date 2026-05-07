#include "mapping.hpp"

#include <random>
#include <limits>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

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

// Total-least-squares line fit on a set of points. Returns unit
// direction d (along the line) and unit normal n (perpendicular),
// plus centroid. The normal is the eigenvector of the smaller
// eigenvalue of the centered 2x2 covariance.
struct LineFit {
    double cx, cy;   // centroid
    double dx, dy;   // unit direction
    double nx, ny;   // unit normal
};

LineFit refit_tls(const std::vector<Point>& pts,
                  const std::vector<int>& idx) {
    LineFit f{0, 0, 1, 0, 0, 1};
    const size_t n = idx.size();
    if (n == 0) return f;

    double sx = 0.0, sy = 0.0;
    for (size_t i = 0; i < n; ++i) {
        sx += pts[idx[i]].x;
        sy += pts[idx[i]].y;
    }
    f.cx = sx / static_cast<double>(n);
    f.cy = sy / static_cast<double>(n);

    double sxx = 0.0, sxy = 0.0, syy = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double dx = pts[idx[i]].x - f.cx;
        const double dy = pts[idx[i]].y - f.cy;
        sxx += dx * dx;
        sxy += dx * dy;
        syy += dy * dy;
    }

    Eigen::Matrix2d C;
    C << sxx, sxy,
         sxy, syy;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(C);
    // eigenvalues in increasing order: col 0 = smaller (normal),
    // col 1 = larger (direction).
    const Eigen::Vector2d nrm = es.eigenvectors().col(0);
    const Eigen::Vector2d dir = es.eigenvectors().col(1);
    f.nx = nrm(0);
    f.ny = nrm(1);
    f.dx = dir(0);
    f.dy = dir(1);
    return f;
}

}  // namespace

std::vector<LineSeg> fit_lines_ransac(const std::vector<Point>& points,
                                      const RansacParams& p) {
    std::vector<LineSeg> lines;
    if (points.size() < static_cast<size_t>(std::max(2, p.min_inliers))) {
        return lines;
    }

    std::vector<Point> remaining = points;
    const size_t initial_size = remaining.size();
    const size_t min_set_size = std::max<size_t>(
        static_cast<size_t>(std::max(2, p.min_inliers)),
        static_cast<size_t>(std::ceil(p.min_inlier_fraction *
                                      static_cast<double>(initial_size))));

    std::mt19937 rng(p.seed != 0 ? p.seed : std::random_device{}());

    while (remaining.size() >= min_set_size &&
           static_cast<int>(lines.size()) < p.max_lines) {

        const int N = static_cast<int>(remaining.size());
        std::uniform_int_distribution<int> dist(0, N - 1);

        std::vector<int> best_inliers;

        for (int trial = 0; trial < p.max_iters_per_line; ++trial) {
            int i = dist(rng);
            int j = dist(rng);
            if (i == j) continue;
            const Point& a = remaining[i];
            const Point& b = remaining[j];
            double dx = b.x - a.x;
            double dy = b.y - a.y;
            double len = std::sqrt(dx * dx + dy * dy);
            if (len < 1e-6) continue;
            // Unit normal (perpendicular to ab).
            const double nx = -dy / len;
            const double ny =  dx / len;
            const double r  = nx * a.x + ny * a.y;

            std::vector<int> trial_inliers;
            trial_inliers.reserve(N / 4);
            for (int k = 0; k < N; ++k) {
                const double d = std::fabs(nx * remaining[k].x +
                                           ny * remaining[k].y - r);
                if (d <= p.inlier_thresh_mm) trial_inliers.push_back(k);
            }
            if (trial_inliers.size() > best_inliers.size()) {
                best_inliers.swap(trial_inliers);
            }
        }

        if (static_cast<int>(best_inliers.size()) < p.min_inliers) break;
        if (best_inliers.size() < min_set_size) break;

        // Refit with total least squares on inliers.
        LineFit fit = refit_tls(remaining, best_inliers);

        // Tighten inlier set against the refit line. This catches points
        // the random-sample line missed and drops any that drift after refit.
        std::vector<int> refined;
        refined.reserve(best_inliers.size());
        for (int k = 0; k < N; ++k) {
            const double d = std::fabs(fit.nx * (remaining[k].x - fit.cx) +
                                       fit.ny * (remaining[k].y - fit.cy));
            if (d <= p.inlier_thresh_mm) refined.push_back(k);
        }
        if (static_cast<int>(refined.size()) < p.min_inliers) {
            // The refit moved enough that we lost the line. Walk away.
            // Drop the original inliers anyway so we don't loop forever.
            std::vector<bool> kill(N, false);
            for (int idx : best_inliers) kill[idx] = true;
            std::vector<Point> next;
            next.reserve(N - best_inliers.size());
            for (int k = 0; k < N; ++k) if (!kill[k]) next.push_back(remaining[k]);
            remaining.swap(next);
            continue;
        }

        // Endpoints: project refined inliers onto direction d, take min/max.
        double t_min =  std::numeric_limits<double>::infinity();
        double t_max = -std::numeric_limits<double>::infinity();
        for (int idx : refined) {
            const double t = fit.dx * (remaining[idx].x - fit.cx) +
                             fit.dy * (remaining[idx].y - fit.cy);
            if (t < t_min) t_min = t;
            if (t > t_max) t_max = t;
        }
        const double ax = fit.cx + t_min * fit.dx;
        const double ay = fit.cy + t_min * fit.dy;
        const double bx = fit.cx + t_max * fit.dx;
        const double by = fit.cy + t_max * fit.dy;

        // Signed distance r so that n . p = r on the line.
        double nx = fit.nx;
        double ny = fit.ny;
        double r  = nx * fit.cx + ny * fit.cy;
        // Convention: normal points away from the sensor at the origin,
        // matching the original rectangle code.
        if (r < 0.0) { nx = -nx; ny = -ny; r = -r; }

        LineSeg seg;
        seg.nx = static_cast<float>(nx);
        seg.ny = static_cast<float>(ny);
        seg.r  = static_cast<float>(r);
        seg.ax = static_cast<float>(ax);
        seg.ay = static_cast<float>(ay);
        seg.bx = static_cast<float>(bx);
        seg.by = static_cast<float>(by);
        lines.push_back(seg);

        // Remove refined inliers from the working set.
        std::vector<bool> kill(N, false);
        for (int idx : refined) kill[idx] = true;
        std::vector<Point> next;
        next.reserve(N - refined.size());
        for (int k = 0; k < N; ++k) if (!kill[k]) next.push_back(remaining[k]);
        remaining.swap(next);
    }

    return lines;
}
