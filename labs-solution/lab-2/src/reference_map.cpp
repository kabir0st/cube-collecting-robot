#include <reference_map.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>

namespace {

const float PI_F = 3.14159265358979323846f;

struct Pt2 { float x, y; };

float median_int(std::vector<int> v) {
    std::sort(v.begin(), v.end());
    const std::size_t n = v.size();
    if (n & 1u) return static_cast<float>(v[n / 2]);
    return 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

float median_float(std::vector<float> v) {
    std::sort(v.begin(), v.end());
    const std::size_t n = v.size();
    if (n == 0) return 0.0f;
    if (n & 1u) return v[n / 2];
    return 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

std::vector<Pt2> bins_to_points(const std::map<int, std::vector<int>>& bins,
                                int min_samples_per_bin) {
    std::vector<Pt2> out;
    out.reserve(bins.size());
    for (auto it = bins.begin(); it != bins.end(); ++it) {
        const std::vector<int>& s = it->second;
        if (static_cast<int>(s.size()) < min_samples_per_bin) continue;
        const float r = median_int(s);
        if (r <= 0.0f) continue;
        const float a = it->first * (PI_F / 180.0f);
        Pt2 p;
        p.x = r * std::cos(a);
        p.y = r * std::sin(a);
        out.push_back(p);
    }
    return out;
}

float bbox_area(const std::vector<Pt2>& pts, float alpha) {
    const float c = std::cos(-alpha), s = std::sin(-alpha);
    float xmin =  1e30f, xmax = -1e30f;
    float ymin =  1e30f, ymax = -1e30f;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        const float x = c * pts[i].x - s * pts[i].y;
        const float y = s * pts[i].x + c * pts[i].y;
        if (x < xmin) xmin = x;
        if (x > xmax) xmax = x;
        if (y < ymin) ymin = y;
        if (y > ymax) ymax = y;
    }
    return (xmax - xmin) * (ymax - ymin);
}

float best_orientation(const std::vector<Pt2>& pts) {
    float best_alpha = 0.0f, best_area = 1e30f;
    for (int deg = 0; deg < 90; ++deg) {
        const float alpha = deg * (PI_F / 180.0f);
        const float area = bbox_area(pts, alpha);
        if (area < best_area) { best_area = area; best_alpha = alpha; }
    }
    const float step = 0.1f * (PI_F / 180.0f);
    const float lo = best_alpha - (PI_F / 180.0f);
    for (int k = 0; k <= 20; ++k) {
        const float alpha = lo + k * step;
        const float area = bbox_area(pts, alpha);
        if (area < best_area) { best_area = area; best_alpha = alpha; }
    }
    return best_alpha;
}

LineSeg make_wall(float nx_r, float ny_r, float r_r,
                  float ax_r, float ay_r, float bx_r, float by_r,
                  float alpha) {
    const float c = std::cos(alpha), s = std::sin(alpha);
    LineSeg w;
    w.nx = c * nx_r - s * ny_r;
    w.ny = s * nx_r + c * ny_r;
    w.r  = r_r;
    w.ax = c * ax_r - s * ay_r;
    w.ay = s * ax_r + c * ay_r;
    w.bx = c * bx_r - s * by_r;
    w.by = s * bx_r + c * by_r;
    if (w.r < 0.0f) { w.nx = -w.nx; w.ny = -w.ny; w.r = -w.r; }
    return w;
}

} // namespace

std::vector<LineSeg> ReferenceMapBuilder::build(
        const std::map<int, std::vector<int>>& bins,
        int min_samples_per_bin) const {

    std::vector<LineSeg> walls;
    const std::vector<Pt2> pts = bins_to_points(bins, min_samples_per_bin);
    if (pts.size() < 8) return walls;

    const float alpha = best_orientation(pts);
    const float c = std::cos(-alpha), s = std::sin(-alpha);

    // Rotate all points into "room frame".
    std::vector<Pt2> rp(pts.size());
    float xmin =  1e30f, xmax = -1e30f;
    float ymin =  1e30f, ymax = -1e30f;
    double sx = 0.0, sy = 0.0;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        rp[i].x = c * pts[i].x - s * pts[i].y;
        rp[i].y = s * pts[i].x + c * pts[i].y;
        if (rp[i].x < xmin) xmin = rp[i].x;
        if (rp[i].x > xmax) xmax = rp[i].x;
        if (rp[i].y < ymin) ymin = rp[i].y;
        if (rp[i].y > ymax) ymax = rp[i].y;
        sx += rp[i].x; sy += rp[i].y;
    }
    const float cx = static_cast<float>(sx / pts.size());
    const float cy = static_cast<float>(sy / pts.size());

    // Classify each point by which wall it belongs to (whichever of the four
    // bbox sides it is closest to). Use median of perpendicular coordinate
    // per cluster as the wall position — robust to far-return outliers that
    // would otherwise push bbox extrema outward.
    std::vector<float> left_x, right_x, bottom_y, top_y;
    left_x.reserve(pts.size() / 2);
    right_x.reserve(pts.size() / 2);
    bottom_y.reserve(pts.size() / 2);
    top_y.reserve(pts.size() / 2);
    for (std::size_t i = 0; i < rp.size(); ++i) {
        const float dl = rp[i].x - xmin;
        const float dr = xmax - rp[i].x;
        const float db = rp[i].y - ymin;
        const float dt = ymax - rp[i].y;
        float m = dl; int side = 0;
        if (dr < m) { m = dr; side = 1; }
        if (db < m) { m = db; side = 2; }
        if (dt < m) { m = dt; side = 3; }
        switch (side) {
            case 0: left_x.push_back(rp[i].x);   break;
            case 1: right_x.push_back(rp[i].x);  break;
            case 2: bottom_y.push_back(rp[i].y); break;
            case 3: top_y.push_back(rp[i].y);    break;
        }
    }
    if (left_x.empty() || right_x.empty() || bottom_y.empty() || top_y.empty()) {
        return walls;
    }

    const float wx_min = median_float(left_x);
    const float wx_max = median_float(right_x);
    const float wy_min = median_float(bottom_y);
    const float wy_max = median_float(top_y);

    // Sanity: the median walls must enclose the centroid.
    if (!(wx_min < cx && cx < wx_max && wy_min < cy && cy < wy_max)) {
        return walls;
    }

    walls.reserve(4);
    walls.push_back(make_wall(-1.0f, 0.0f, -wx_min, wx_min, wy_min, wx_min, wy_max, alpha));
    walls.push_back(make_wall( 1.0f, 0.0f,  wx_max, wx_max, wy_min, wx_max, wy_max, alpha));
    walls.push_back(make_wall(0.0f, -1.0f, -wy_min, wx_min, wy_min, wx_max, wy_min, alpha));
    walls.push_back(make_wall(0.0f,  1.0f,  wy_max, wx_min, wy_max, wx_max, wy_max, alpha));

    return walls;
}
