#pragma once
#include <opencv2/opencv.hpp>
#include <types.hpp>

class CalibrateCamera {
    private:
        CameraConfig config;
        static const int k_neighbors = 5;
        cv::Scalar hsvToRgb(float h, float s, float v);
        void display_color_in_terminal(const cv::Scalar& hsv_color);
        std::pair<cv::Scalar, cv::Mat> get_dominant_color(const cv::Mat& frame, const cv::Rect& roi);

    public:
        CalibrateCamera(const CameraConfig &config);
        cv::Scalar start_calibration();
};