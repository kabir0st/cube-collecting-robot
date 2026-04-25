#pragma once
#include <opencv2/opencv.hpp>

// color tracking margins
static constexpr float HUE_MARGIN  = 10.f;  // hue is sensitive
static constexpr float SAT_MARGIN  = 40.f;  // sat can vary a lot in lighting
static constexpr float VAL_MARGIN  = 40.f;  // val changes with shadows/light


struct CameraConfig {
    int width;
    int height;
    int fps_target;
    CameraConfig(int w, int h, int f);
};

struct FilterResult {
    cv::Scalar dominant_color;
    cv::Mat mask;
    bool ready = false;
};
