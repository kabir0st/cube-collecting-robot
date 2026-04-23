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

cv::Scalar CalibrateCamera::get_dominant_color(const cv::Mat& frame) {
    int w = frame.cols / 2;
    int h = frame.rows / 2;
    int x = (frame.cols - w) / 2;
    int y = (frame.rows - h) / 2;
    const cv::Rect roi = cv::Rect(x, y, w, h);
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
    return cv::Scalar(
        centers.at<float>(dominant_idx, 0),
        centers.at<float>(dominant_idx, 1),
        centers.at<float>(dominant_idx, 2));
}

cv::Scalar CalibrateCamera::start_calibration() {
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        throw runtime_error("Cannot Open camera.");
    }
    
    cap.set(cv::CAP_PROP_FRAME_WIDTH, config.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, config.height);
    cap.set(cv::CAP_PROP_FPS, config.fps_target);
    
    cv::Mat frame;
    cv::Scalar main_color_hsv;
    
    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        
        main_color_hsv = get_dominant_color(frame);
        display_color_in_terminal(main_color_hsv);
        
        cv::rectangle(frame, 
                    cv::Point(frame.cols/4, frame.rows/4), 
                    cv::Point(3*frame.cols/4, 3*frame.rows/4), 
                    cv::Scalar(0, 255, 0), 2);
        
        char key = cv::waitKey(1);
        if (key == 'q') {
            cout << "\nQuitting without selecting color" << endl;
            break;
        }
        else if (key == 'c') {
            cout << "\n\nColor captured!" << endl;
            cout << "Selected HSV: (" << (int)main_color_hsv[0] << ", " 
                 << (int)main_color_hsv[1] << ", " << (int)main_color_hsv[2] << ")" << endl;
            
            break;
        }
    }

    cv::destroyAllWindows();
    return main_color_hsv;
}