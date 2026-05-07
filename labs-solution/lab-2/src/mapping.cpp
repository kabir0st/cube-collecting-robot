#include <mapping.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>


// =============================================================================
// Total-least-squares line fit (orthogonal regression)
// =============================================================================
//
// Given a set of points, find the line that minimises the sum of squared
// PERPENDICULAR distances. The classical y = mx + b least-squares fit
// minimises VERTICAL distances and breaks down for vertical walls.
// TLS is symmetric in x and y and works for any orientation.
//
// Method: form the 2x2 sample covariance about the centroid; the
// eigenvector of the smaller eigenvalue is the line normal, the
// eigenvector of the larger eigenvalue is the line direction.
namespace {

struct LineFit {
    double cx, cy;   // centroid
    double dx, dy;   // unit direction along the line
    double nx, ny;   // unit normal, perpendicular to (dx, dy)
};

LineFit refit_tls(const std::vector<Point>& pts,
                  const std::vector<int>& idx) {
    const size_t n = idx.size();

    double sx = 0.0, sy = 0.0;
    for (size_t i = 0; i < n; ++i) {
        sx += pts[idx[i]].x;
        sy += pts[idx[i]].y;
    }
    const double cx = sx / static_cast<double>(n);
    const double cy = sy / static_cast<double>(n);

    double sxx = 0.0, sxy = 0.0, syy = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double dx = pts[idx[i]].x - cx;
        const double dy = pts[idx[i]].y - cy;
        sxx += dx * dx;
        sxy += dx * dy;
        syy += dy * dy;
    }

    Eigen::Matrix2d C;
    C << sxx, sxy,
         sxy, syy;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(C);

    // SelfAdjointEigenSolver returns eigenvalues in ascending order:
    //   col(0) = smaller -> normal
    //   col(1) = larger  -> direction
    LineFit f;
    f.cx = cx; f.cy = cy;
    f.nx = es.eigenvectors()(0, 0);
    f.ny = es.eigenvectors()(1, 0);
    f.dx = es.eigenvectors()(0, 1);
    f.dy = es.eigenvectors()(1, 1);
    return f;
}


// Inliers of a candidate line (n, r) within `thresh` perpendicular distance.
std::vector<int> inliers_of(const std::vector<Point>& pts,
                            double nx, double ny, double r,
                            double thresh) {
    std::vector<int> out;
    out.reserve(pts.size() / 4);
    for (size_t k = 0; k < pts.size(); ++k) {
        if (std::fabs(nx * pts[k].x + ny * pts[k].y - r) <= thresh)
            out.push_back(static_cast<int>(k));
    }
    return out;
}


// Replace `pts` with everything that is NOT in `inliers`. O(n) and
// stable (preserves order of survivors).
void erase_inliers(std::vector<Point>& pts,
                   const std::vector<int>& inliers) {
    std::vector<bool> mark(pts.size(), false);
    for (int i : inliers) mark[i] = true;
    std::vector<Point> next;
    next.reserve(pts.size() - inliers.size());
    for (size_t k = 0; k < pts.size(); ++k)
        if (!mark[k]) next.push_back(pts[k]);
    pts.swap(next);
}

}  // namespace


std::vector<LineSeg> fit_lines_ransac(const std::vector<Point>& points,
                                      const RansacParams& p) {
    std::vector<LineSeg> walls;
    if (points.size() < static_cast<size_t>(std::max(2, p.min_inliers)))
        return walls;

    std::vector<Point> remaining = points;
    const size_t initial = remaining.size();

    // Combined floor: at least min_inliers AND min_inlier_fraction of the
    // ORIGINAL cloud size. Using the original size (not the dwindling
    // remaining size) prevents the loop from extracting noise lines once
    // the real walls have been consumed.
    const size_t hard_floor = std::max<size_t>(
        static_cast<size_t>(std::max(2, p.min_inliers)),
        static_cast<size_t>(std::ceil(p.min_inlier_fraction * initial)));

    std::mt19937 rng(p.seed != 0 ? p.seed : std::random_device{}());

    while (remaining.size() >= hard_floor &&
           static_cast<int>(walls.size()) < p.max_lines) {

        const int N = static_cast<int>(remaining.size());
        std::uniform_int_distribution<int> pick(0, N - 1);

        // -- Inner RANSAC loop: best 2-point hypothesis ----------------
        std::vector<int> best;
        for (int trial = 0; trial < p.max_iters_per_line; ++trial) {
            int i = pick(rng);
            int j = pick(rng);
            if (i == j) { --trial; continue; }    // resample, don't waste

            const double dx  = remaining[j].x - remaining[i].x;
            const double dy  = remaining[j].y - remaining[i].y;
            const double len = std::hypot(dx, dy);
            if (len < 1e-6) continue;

            const double nx = -dy / len;
            const double ny =  dx / len;
            const double r  = nx * remaining[i].x + ny * remaining[i].y;

            std::vector<int> inl = inliers_of(remaining, nx, ny, r,
                                              p.inlier_thresh_mm);
            if (inl.size() > best.size()) best.swap(inl);
        }
        if (static_cast<int>(best.size()) < p.min_inliers) break;

        // -- Refit on inliers, then re-collect inliers around the refit -
        const LineFit fit = refit_tls(remaining, best);
        const double r_refit = fit.nx * fit.cx + fit.ny * fit.cy;
        const std::vector<int> refined = inliers_of(remaining,
                                                    fit.nx, fit.ny, r_refit,
                                                    p.inlier_thresh_mm);
        if (static_cast<int>(refined.size()) < p.min_inliers) {
            // Refit drifted too far; drop original inliers and try again.
            erase_inliers(remaining, best);
            continue;
        }

        // -- Endpoints from min/max projection along the line direction -
        double t_min =  std::numeric_limits<double>::infinity();
        double t_max = -std::numeric_limits<double>::infinity();
        for (int k : refined) {
            const double t = fit.dx * (remaining[k].x - fit.cx)
                           + fit.dy * (remaining[k].y - fit.cy);
            if (t < t_min) t_min = t;
            if (t > t_max) t_max = t;
        }
        // Pad the segment a little so a downstream endpoint test (Cox
        // matcher) covers points hitting just past the last inlier.
        t_min -= p.endpoint_padding_mm;
        t_max += p.endpoint_padding_mm;

        // -- Build the LineSeg with normal pointing away from origin ----
        double nx = fit.nx, ny = fit.ny;
        double r  = nx * fit.cx + ny * fit.cy;
        if (r < 0.0) { nx = -nx; ny = -ny; r = -r; }

        LineSeg seg;
        seg.nx = static_cast<float>(nx);
        seg.ny = static_cast<float>(ny);
        seg.r  = static_cast<float>(r);
        seg.ax = static_cast<float>(fit.cx + t_min * fit.dx);
        seg.ay = static_cast<float>(fit.cy + t_min * fit.dy);
        seg.bx = static_cast<float>(fit.cx + t_max * fit.dx);
        seg.by = static_cast<float>(fit.cy + t_max * fit.dy);
        walls.push_back(seg);

        erase_inliers(remaining, refined);
    }

    return walls;
}
