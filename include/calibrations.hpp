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
    bool has(const std::string& key) const;
    // i want to set default values too if config key is not found 
    std::string get_str (const std::string& key, const std::string& def = "") const;
    int         get_int (const std::string& key, int    def = 0)    const;
    double      get_dbl (const std::string& key, double def = 0.0)  const;
    bool        get_bool(const std::string& key, bool   def = false) const;

    const std::map<std::string, std::string>& raw() const { return _kv; }

    void set(const std::string& key, const std::string& value) { _kv[key] = value; }

private:
    std::map<std::string, std::string> _kv;
};

bool load_config(const std::string& path, Config& out);

bool load_filter_result(const Config& cfg, FilterResult& out);

bool save_filter_section(const std::string& path,
                         const FilterResult& target,
                         bool ready);

double get_lidar_sensor_noise(LidarData &lidar_data);

std::vector<LineSeg> find_walls(LidarData &lidar_data,
                                const RansacParams& params);

