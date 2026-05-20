#include <velprofile.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

void apply_trap_profile(std::vector<PathPoint>& path,
                        double v_cruise_mm_s,
                        double accel_mm_s2,
                        double min_v_mm_s) {
    const size_t N = path.size();
    if (N == 0 || accel_mm_s2 <= 0.0 || v_cruise_mm_s <= 0.0) return;
    // Cumulative arc length per point.
    std::vector<double> s(N, 0.0);
    for (size_t i = 1; i < N; ++i) {
        const double dx = path[i].x - path[i - 1].x;
        const double dy = path[i].y - path[i - 1].y;
        s[i] = s[i - 1] + std::hypot(dx, dy);
    }
    const double S_total = s.back();
    if (S_total <= 0.0) return;
    const double floor_v = std::max(0.0, min_v_mm_s);
    for (size_t i = 0; i < N; ++i) {
        const double v_up   = std::sqrt(2.0 * accel_mm_s2 * s[i]);
        const double v_down = std::sqrt(2.0 * accel_mm_s2 * std::max(0.0, S_total - s[i]));
        const double v_trap = std::min({v_cruise_mm_s, v_up, v_down});
        path[i].v_max = std::max(floor_v, v_trap);
    }
}
