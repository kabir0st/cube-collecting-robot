#include <iostream>
#include <types.hpp>
#include <calibrate.hpp>
#include <server.hpp>
#include <utils.hpp>
#include <cmath>
#include <numeric>
#include <mapping.hpp>

//   σ = sqrt (Σ(xᵢ - μ)^2/N)
float LidarCalibration::calibrate() {
    std::cout << "Points collected " << scan_collection.size();
    if (scan_collection.empty()) {
        throw std::runtime_error("No points collected for calibration.");
    }
    
    double accumulated_error = 0.0;
    int processed_points = 0;
    
    for (const auto &pair : scan_collection) {
        const auto& distances = pair.second;
        if (distances.empty()) {
            continue;
        }
        // One-pass algorithm for both mean and variance
        double mean = 0.0;
        // Sum of squared differences
        double M2 = 0.0;  
        size_t n = 0;
        for (int d : distances) {
            n++;
            double delta = d - mean;
            mean += delta / n;
            double delta2 = d - mean;
            M2 += delta * delta2;
        }   
        // M2 now contains the sum of squared differences
        accumulated_error += M2;
        processed_points += static_cast<int>(n);
    }   
    // RMSE (Root Mean Square Error)
    return static_cast<float>(std::sqrt(accumulated_error / processed_points));
}


void LidarCalibration::process_point(int quality, int angle, int distance) {
    if (quality > 0) {
        if (angle == 0) {
            std::cout << "\r Angle 0 distance : " << distance << std::flush;
        }
        scan_collection[angle].push_back(distance);
    }
}

void LidarCalibration::create_reference_map() {
    std::cout << "[info] Creating reference map" << std::endl;
    std::vector<Point> scan;

    // Keep map geometry in millimeters to match downstream scan matching.

    for (const auto& pair: scan_collection){
        const auto &angle = pair.first;
        const auto &distances = pair.second;
        if (distances.size() == 0) continue;
        const double avg = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
        const double angle_rad = static_cast<double>(angle) * M_PI / 180.0;
        const double distance_mm = avg;
        // x = d * cos theta
        // y = d * sin theta
        scan.emplace_back(distance_mm * std::cos(angle_rad), distance_mm * std::sin(angle_rad));
    }
    // std::vector<Line> walls = extractor.extractWalls(scan);
    std::cout << "[info] Walls payena ? " << scan.size() << std::endl;

}
