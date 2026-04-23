#pragma once
#include <opencv2/opencv.hpp>
#include <include/types.hpp>

class CalibrateCamera {
    private:
        CameraConfig config;
        static const int k_neighbors = 5;
        cv::Scalar hsvToRgb(float h, float s, float v);
        void display_color_in_terminal(const cv::Scalar& hsv_color);
        cv::Scalar get_dominant_color(const cv::Mat& frame);
    
    public:
        CalibrateCamera(const CameraConfig &config);
        cv::Scalar start_calibration();
}