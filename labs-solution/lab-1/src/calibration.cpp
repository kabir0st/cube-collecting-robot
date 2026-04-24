#include "calibration.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <vector>
#include <algorithm>

using namespace std;

CameraConfig::CameraConfig(int w, int h, int f) : 
    width(w), height(h), fps_target(f) {}

CalibrateCamera::CalibrateCamera(const CameraConfig& cfg) : config(cfg) {}

cv::Scalar CalibrateCamera::hsvToRgb(float h, float s, float v) {
    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(h, s, v));
    cv::Mat rgb;
    cv::cvtColor(hsv, rgb, cv::COLOR_HSV2RGB);
    return cv::Scalar(rgb.at<cv::Vec3b>(0,0)[0], 
                     rgb.at<cv::Vec3b>(0,0)[1], 
                     rgb.at<cv::Vec3b>(0,0)[2]);
}

void CalibrateCamera::display_color_in_terminal(const cv::Scalar& hsv_color) {
    cv::Scalar rgb = hsvToRgb(hsv_color[0], hsv_color[1], hsv_color[2]);
    
    std::cout << "\r\033[K";
    std::cout << "Dominant Color: ";
    
    std::cout << "\033[48;2;" << (int)rgb[0] << ";" << (int)rgb[1] << ";" << (int)rgb[2] << "m";
    std::cout << "      ";
    std::cout << "\033[0m ";
    
    std::cout << "HSV(" << (int)hsv_color[0] << ", " 
              << (int)hsv_color[1] << ", " << (int)hsv_color[2] << ") ";
    std::cout << "RGB(" << (int)rgb[0] << "," << (int)rgb[1] << "," << (int)rgb[2] << ")";
    std::cout << std::flush;
}





std::pair<cv::Scalar, cv::Mat> CalibrateCamera::get_dominant_color(const cv::Mat& frame, const cv::Rect& roi) {
    CalibrationResult result;
    cv::Mat roi_frame = frame(roi);
    cv::Mat hsv_frame;
    cv::cvtColor(roi_frame, hsv_frame, cv::COLOR_BGR2HSV);
    
    cv::Mat pixels = hsv_frame.reshape(1, hsv_frame.rows * hsv_frame.cols);
    pixels.convertTo(pixels, CV_32F);
    cv::Mat labels, centers;
    cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 10, 1.0);
    cv::kmeans(pixels, k_neighbors, labels, criteria, 10, cv::KMEANS_RANDOM_CENTERS, centers);
    
    std::vector<int> counts(k_neighbors, 0);
    for (int i = 0; i < labels.rows; i++) {
        counts[labels.at<int>(i)]++;
    }
    
    int dominant_idx = std::max_element(counts.begin(), counts.end()) - counts.begin();

    //labels 
    // pixels


    float h_min = 180.f, s_min = 255.f, v_min = 255.f;
    float h_max = 0.f, s_max = 0.f, v_max = 0.f;
    std::vector<float> h_vals;
    h_vals.reserve(counts[dominant_idx]);
    for (int i = 0; i < labels.rows; i++) {
        if (labels.at<int>(i) == dominant_idx) {
            float h = pixels.at<float>(i, 0);
            float s = pixels.at<float>(i, 1);
            float v = pixels.at<float>(i, 2);
            h_vals.push_back(h);
            h_min = min(h_min, h);
            s_min = min(s_min, s);
            v_min = min(v_min, v);
            h_max = std::max(h_max, h);
            s_max = std::max(s_max, s);
            v_max = std::max(v_max, v);
        }
    }

    const float h_margin = 10.f;
    const float s_margin = 30.f;
    const float v_margin = 40.f;

    cv::Mat mask;
    if ((h_max - h_min) > 90.f){
        sort(h_vals.begin(), h_vals.end());
        float max_gap = 0.f;
        int gap_idx = 1;
        for (int i = 1; i < (int)h_vals.size(); i++) {
            float gap = h_vals[i] - h_vals[i - 1];
            if (gap > max_gap) {
                max_gap = gap;
                gap_idx = i;
            }
        }

        float h_low_max = h_vals[gap_idx - 1];
        float h_high_min = h_vals[gap_idx];

        // High side: ~170 → 180
        cv::Scalar lower1(std::max(0.f,   h_high_min - h_margin), std::max(0.f,   s_min - s_margin), std::max(0.f,   v_min - v_margin));
        cv::Scalar upper1(180.f,                                   std::min(255.f, s_max + s_margin), std::min(255.f, v_max + v_margin));

        // Low side: 0 → ~8
        cv::Scalar lower2(0.f,                                     std::max(0.f,   s_min - s_margin), std::max(0.f,   v_min - v_margin));
        cv::Scalar upper2(std::min(180.f, h_low_max + h_margin),   std::min(255.f, s_max + s_margin), std::min(255.f, v_max + v_margin));

        cv::Mat mask1, mask2;
        cv::inRange(hsv_frame, lower1, upper1, mask1);
        cv::inRange(hsv_frame, lower2, upper2, mask2);
        cv::bitwise_or(mask1, mask2, mask);
    } else {
        cv::Scalar lower(std::max(0.f,   h_min - h_margin),
                         std::max(0.f,   s_min - s_margin),
                         std::max(0.f,   v_min - v_margin));
        cv::Scalar upper(std::min(180.f, h_max + h_margin),
                         std::min(255.f, s_max + s_margin),
                         std::min(255.f, v_max + v_margin));
        cv::inRange(hsv_frame, lower, upper, mask);
    }
    cv::Scalar dominant(
        centers.at<float>(dominant_idx, 0),
        centers.at<float>(dominant_idx, 1),
        centers.at<float>(dominant_idx, 2));
    return {dominant, mask};
}

cv::Scalar CalibrateCamera::start_calibration() {
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        throw runtime_error("Cannot Open camera.");
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, config.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, config.height);
    cap.set(cv::CAP_PROP_FPS, config.fps_target);
    int w = config.width / 2;
    int h = config.height / 2;
    int x = (config.width - w) / 2;
    int y = (config.height - h) / 2;
    const cv::Rect roi = cv::Rect(x, y, w, h);
    cv::Mat frame;
    cv::Scalar dominant_color;
    while (true)
    {
        cap >> frame;
        if (frame.empty()) break;
        
        auto [dominant_color, mask] = get_dominant_color(frame, roi);
        display_color_in_terminal(dominant_color);
        
        cv::rectangle(frame, 
                    roi,
                    cv::Scalar(0, 255, 0), 2);
        cv::imshow("Calibration Setup", frame);
        char key = cv::waitKey(1);
        if (key == 'q') {
            cout << "\nQuitting without selecting color" << endl;
            break;
        }
        else if (key == 'c') {
            cout << "\n\nColor captured!" << endl;
            cout << "Selected HSV: (" << (int)dominant_color[0] << ", " 
                 << (int)dominant_color[1] << ", " << (int)dominant_color[2] << ")" << endl;
            break;
        }
    }
    cv::destroyAllWindows();
    return dominant_color;
}