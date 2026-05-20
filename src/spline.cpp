#include <spline.hpp>

#include <velprofile.hpp>

#include <cstddef>
#include <cmath>

namespace {
    // B-cubic basis matrix B (uniform, scaled by 1/6).
    //
    //                  [-1  3 -3  1]
    //   B = (1/6) *    [ 3 -6  3  0]
    //                  [-3  0  3  0]
    //                  [ 1  4  1  0]
    //
    // S(t) = [t^3, t^2, t, 1] * B * [P_{i-1}, P_i, P_{i+1}, P_{i+2}]^T
    inline double b3_blend(double t, double a, double b, double c, double d) {
        const double t2 = t * t;
        const double t3 = t2 * t;
        const double w_a = (-t3 + 3.0 * t2 - 3.0 * t + 1.0) / 6.0;
        const double w_b = ( 3.0 * t3 - 6.0 * t2          + 4.0) / 6.0;
        const double w_c = (-3.0 * t3 + 3.0 * t2 + 3.0 * t + 1.0) / 6.0;
        const double w_d = ( t3 ) / 6.0;
        return w_a * a + w_b * b + w_c * c + w_d * d;
    }
}

std::vector<PathPoint> bcubic_spline(
    const std::vector<PathPoint>& ctrl,
    int samples_per_segment,
    double default_v_max,
    double default_margin,
    double accel_mm_s2,
    double min_v_mm_s) {

    std::vector<PathPoint> out;
    if (ctrl.size() < 2 || samples_per_segment < 1) {
        return out;
    }

    // Phantom-pad: two extra copies of the first and last control points
    // at each end (giving multiplicity 3 in the knot vector). This is what
    // makes the cubic B-spline actually interpolate the endpoints — with
    // only one phantom you'd get S(0) = (5*F + P1)/6, not F.
    std::vector<PathPoint> padded;
    padded.reserve(ctrl.size() + 4);
    padded.push_back(ctrl.front());
    padded.push_back(ctrl.front());
    for (const auto& p : ctrl) padded.push_back(p);
    padded.push_back(ctrl.back());
    padded.push_back(ctrl.back());

    out.reserve((padded.size() - 3) * static_cast<size_t>(samples_per_segment + 1));

    // For each interior 4-tuple of padded controls, sample the cubic.
    for (size_t i = 0; i + 3 < padded.size(); ++i) {
        const PathPoint& p0 = padded[i + 0];
        const PathPoint& p1 = padded[i + 1];
        const PathPoint& p2 = padded[i + 2];
        const PathPoint& p3 = padded[i + 3];

        // Skip t=1 of each segment except the last; the next segment's t=0
        // produces the same point, so dropping the duplicate avoids dense
        // pairs in the output.
        const bool last_segment = (i + 4 == padded.size());
        const int  s_end = last_segment ? samples_per_segment : samples_per_segment - 1;

        for (int s = 0; s <= s_end; ++s) {
            const double t = static_cast<double>(s) / samples_per_segment;
            PathPoint pp;
            pp.x      = b3_blend(t, p0.x, p1.x, p2.x, p3.x);
            pp.y      = b3_blend(t, p0.y, p1.y, p2.y, p3.y);
            pp.theta  = 0.0;  // filled in below by tangent pass
            pp.v_max  = default_v_max;
            pp.margin = default_margin;
            out.push_back(pp);
        }
    }

    // Tangent direction at each output point.
    if (out.size() >= 2) {
        for (size_t i = 0; i < out.size(); ++i) {
            double dx, dy;
            if (i == 0) {
                dx = out[1].x - out[0].x;
                dy = out[1].y - out[0].y;
            } else if (i == out.size() - 1) {
                dx = out[i].x - out[i - 1].x;
                dy = out[i].y - out[i - 1].y;
            } else {
                dx = out[i + 1].x - out[i - 1].x;
                dy = out[i + 1].y - out[i - 1].y;
            }
            if (dx == 0.0 && dy == 0.0) {
                // degenerate; preserve previous theta if available
                out[i].theta = (i > 0) ? out[i - 1].theta : 0.0;
            } else {
                out[i].theta = std::atan2(dy, dx);
            }
        }
    }

    // The very last output point uses the caller-specified final
    // orientation rather than the path tangent.
    if (!out.empty()) {
        out.back().theta = ctrl.back().theta;
    }

    apply_trap_profile(out, default_v_max, accel_mm_s2, min_v_mm_s);

    return out;
}
