#pragma once
#include <opencv2/opencv.hpp>
#include <types.hpp>

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
        void display_color_in_terminal(const cv::Scalar& hsv_color);
        cv::Scalar get_dominant_color(const cv::Mat& frame, const cv::Rect& roi);
        bool create_filter_mask();

    public:
        CalibrateCamera(const CameraConfig &config);
        FilterResult start_filter_calibration();
};