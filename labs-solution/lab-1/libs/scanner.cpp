#include <vector>
#include <iostream>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <vector>

#include <opencv2/opencv.hpp>

using namespace std;

struct CameraConfig {
    int width;
    int height;
    int fps_target;
    
    CameraConfig(int w, int h, int f): 
    width(w), height(h), fps_target(f) {}
};

class ObjectColorFinder{
    private:
        CameraConfig config;
        cv::Scalar obj_color;
    
    public:
        cv::Scalar color_picker(){
            cv::VideoCapture cap(0, cv::CAP_V4L2);
            if (!cap.isOpened()){
                throw runtime_error("Cannot Open camera.");
            };
            cap.set(cv::CAP_PROP_FRAME_WIDTH, config.width);
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, config.height);
            cv::Mat kernel = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(5, 5)
            );

        vector<vector<cv::Point>> contours;
        cv::Mat frame, hsv, mask;

        auto fps_start = chrono::steady_clock::now();
        int frames, cx, cy, fps = 0;
            
        while(1) {

        }

        }
};