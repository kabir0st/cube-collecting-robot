#pragma once
#include <opencv2/opencv.hpp>
#include <types.hpp>

using namespace std;
using namespace cv;

class CalibrateCamera {
    private:
        CameraConfig config;
        static const int k_neighbors = 5;
        // share variables for the function;
        int dominant_idx;
        int d_index_pixel_count;
        Mat hsv_frame;
        Mat pixels;
        Mat labels;
        Mat centers;
        Mat mask;

        Scalar hsvToRgb(float h, float s, float v);
        void display_color_in_terminal(const Scalar& hsv_color);
        Scalar get_dominant_color(const Mat& frame, const Rect& roi);
        bool create_filter_mask(FilterResult& result);

    public:
        CalibrateCamera(const CameraConfig &config);
        FilterResult start_filter_calibration();
};