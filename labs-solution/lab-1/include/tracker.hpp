#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>
#include <stdio.h>
#include <types.hpp>

using namespace std;
using namespace cv;

class TrackerClass {
    CameraConfig config;

    public:
        TrackerClass(const CameraConfig& cfg): config(cfg) {}
        void start(FilterResult& filter, TrackerResult &result);  
};