#pragma once
#include <cmath>
#include <map>
#include <mutex>
#include <vector>

#include <types.hpp>

#include <opencv2/opencv.hpp>

// lidar calibration
double get_lidar_sensor_noise(LidarData &lidar_data);

// color hsb calibration
class CalibrateCamera {
    private:
        CameraConfig config;
        static const int k_neighbors = 5;
        // share variables for the function;
        int dominant_idx;
        int d_index_pixel_count;
        cv::Mat hsv_frame;
        cv::Mat pixels;
        cv::Mat labels;
        cv::Mat centers;
        cv::Mat mask;

        cv::Scalar hsvToRgb(float h, float s, float v);
        cv::Scalar get_dominant_color(const cv::Mat& frame, const cv::Rect& roi);
        bool create_filter_mask(FilterResult& result);

    public:
        CalibrateCamera(const CameraConfig &config);
        FilterResult start_filter_calibration();
};

class Config {
public:
    // True if the given dotted key was present in the file.
    bool has(const std::string& key) const;
    // return defaults if config value is not found
    std::string get_str (const std::string& key, const std::string& def = "") const;
    int         get_int (const std::string& key, int    def = 0)    const;
    double      get_dbl (const std::string& key, double def = 0.0)  const;
    bool        get_bool(const std::string& key, bool   def = false) const;

    const std::map<std::string, std::string>& raw() const { return _kv; }

    void set(const std::string& key, const std::string& value) { _kv[key] = value; }

private:
    std::map<std::string, std::string> _kv;
};

// Read a YAML file from disk into `out`. Returns false if the file can't be
// opened; missing file is NOT an error to the caller — they should treat it
// as "all defaults" and proceed. Malformed lines are silently skipped so a
// stray comment / blank line doesn't abort boot.
bool load_config(const std::string& path, Config& out);

// Populate `out` from `cfg`. Returns false if any required filter.* key is
// missing or malformed — caller treats that as "not calibrated yet".
bool load_filter_result(const Config& cfg, FilterResult& out);

// Save just the `filter:` section to `path`. Reads the existing file (if any),
// drops any prior `filter:` block, and appends the new one. All other lines /
// comments are preserved verbatim. Returns false on I/O failure.
bool save_filter_result(const std::string& path, const FilterResult& result);

double get_lidar_sensor_noise(LidarData &lidar_data);

std::vector<LineSeg> find_walls(LidarData &lidar_data,
                                const RansacParams& params);

// POD bag for all `arena:` config keys. Loaded once in main and passed by
// const ref into loop_localization. Tolerances are stored already in mm/rad
// (degrees in the YAML are converted at load time).
struct ArenaConfig {
    bool   in_arena              = true;
    double width_mm              = 2400.0;
    double length_mm             = 3600.0;
    double validation_tol_mm     = 100.0;
    double validation_tol_rad    = 5.0 * M_PI / 180.0;

    // Two RANSAC presets, picked by `in_arena`:
    //   - ransac_arena: clean rectangular arena, ~4 long walls, robot
    //     sitting inside. Tight gates, high inlier requirement, hard cap
    //     at 4 lines.
    //   - ransac_room: real-room operation with furniture / clutter
    //     occluding the walls. Looser gates, lower inlier requirement,
    //     extract more candidates and filter by minimum segment length
    //     so chair / desk edges are rejected.
    RansacParams ransac_arena = []{
        RansacParams p;
        p.inlier_thresh_mm    = 25.0;
        p.inlier_points_req   = 20;
        p.min_inlier_fraction = 0.10;
        p.max_lines           = 4;
        p.min_segment_len_mm  = 500.0;
        return p;
    }();
    RansacParams ransac_room = []{
        RansacParams p;
        p.inlier_thresh_mm    = 40.0;
        p.inlier_points_req   = 10;
        p.min_inlier_fraction = 0.02;
        p.max_lines           = 10;
        p.min_segment_len_mm  = 800.0;
        return p;
    }();
};

// Build the four-segment analytical rectangle: home wall behind the robot at
// x=0, far wall at x=length_mm, side walls at y=±width_mm/2. Normals point
// into the arena so Cox's residual is positive for interior points (matches
// the convention in src/cox.cpp:45).
std::vector<LineSeg> build_arena_map(double width_mm, double length_mm);

// Compare observed RANSAC walls against the analytical rectangle. Returns
// true iff every observed wall matches one of the expected walls within
// the given tolerances. Accepts (n, r) == (-n, -r) (RANSAC always emits
// positive r, two expected walls have negative r). A *missing* expected
// wall is OK (corner occlusion); an *extra* observed wall that matches
// nothing is a fail. Logs unmatched walls.
bool validate_arena_walls(const std::vector<LineSeg>& expected,
                          const std::vector<LineSeg>& observed,
                          double tol_mm, double tol_rad);

// Result of try_fit_arena_fallback: an analytical map in the chosen arena
// frame plus the robot heading inside that frame.
struct ArenaFit {
    std::vector<LineSeg> ref_map;
    double home_theta = 0.0;
    bool   rotated    = false;
};

// Fallback used only after validate_arena_walls rejects every observed wall.
// Groups the four observed walls into two perpendicular pairs, checks the
// pair spans match the arena dimensions (in either rotation), and treats
// the closer short wall as "home". Returns true on dimensional match (out
// is populated, Cox can take over with its relocalize sweep); false when
// the observed scene is not this arena (caller should abort).
bool try_fit_arena_fallback(const std::vector<LineSeg>& observed,
                            const ArenaConfig& arena,
                            ArenaFit& out);

namespace calib {

struct ArenaBounds {
    double min_x = 0.0, max_x = 0.0, min_y = 0.0, max_y = 0.0;
};

void compute_arena_bounds(const std::vector<LineSeg>& walls);
bool get_arena_bounds(ArenaBounds& out);

}  // namespace calib
