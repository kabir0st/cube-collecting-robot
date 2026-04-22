#pragma once
#include <opencv2/opencv.hpp>

struct CameraConfig {
    int width;
    int height;
    int fps_target;
    CameraConfig(int w, int h, int f);
};

class ObjectColorFinder {
private:
    CameraConfig config;
    cv::Scalar dominate_color;
    static const int k_neighbors = 5;
    
    cv::Scalar hsvToRgb(double h, double s, double v);
    void display_color_in_terminal(const cv::Scalar& hsv_color);
    cv::Scalar get_dominant_color(const cv::Mat &frame);

public:
    ObjectColorFinder(const CameraConfig& cfg);
    cv::Scalar color_picker();
};