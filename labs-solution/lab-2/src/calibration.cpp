#include <iostream>
#include <types.hpp>
#include <calibrate.hpp>
#include <server.hpp>
#include <utils.hpp>
#include <cmath>

// Pooled sample standard deviation across per-angle bins:
//   σ = sqrt( Σᵢ Σⱼ (dᵢⱼ − μᵢ)² / Σᵢ (nᵢ − 1) )
// Two-pass within each bin to avoid catastrophic cancellation.
float LidarCalibration::calibrate() {

    double ss_total = 0.0;       // Σᵢ Σⱼ (d − μᵢ)²
    double dof_total = 0.0;      // Σᵢ (nᵢ − 1)

    for (const auto& pair : scan_collection) {
        const auto& distances = pair.second;
        const std::size_t n = distances.size();
        if (n < 2) continue;     // single-sample bins carry no variance info

        double sum = 0.0;
        for (int d : distances) sum += static_cast<double>(d);
        const double mean = sum / static_cast<double>(n);

        double ss = 0.0;
        for (int d : distances) {
            const double diff = static_cast<double>(d) - mean;
            ss += diff * diff;
        }

        ss_total  += ss;
        dof_total += static_cast<double>(n - 1);
    }

    if (dof_total <= 0.0) return 0.0f;
    return static_cast<float>(std::sqrt(ss_total / dof_total));
}

void LidarCalibration::collect_points(int quality, int angle, int distance) {
    if (quality > 0) {
        scan_collection[angle].push_back(distance);
    }
}

