#include <vector>
#include <iostream>
#include <chrono>
#include <thread>

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
        
};