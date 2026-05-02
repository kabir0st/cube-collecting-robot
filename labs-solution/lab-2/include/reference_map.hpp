#pragma once
#include <map>
#include <vector>
#include <types.hpp>

// Builds a 4-wall rectangular reference map from per-angle distance bins
// (the same buffer the calibrator filled while the lidar was stationary).
// Returns 4 LineSeg in the lidar's start-frame, with normals pointing
// AWAY from the sensor origin and r >= 0 by convention.
class ReferenceMapBuilder {
public:
    std::vector<LineSeg> build(const std::map<int, std::vector<int>>& bins,
                               int min_samples_per_bin = 5) const;
};
