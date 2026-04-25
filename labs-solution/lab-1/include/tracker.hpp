#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>
#include <stdio.h>
#include <types.hpp>

class Tracker {
    CameraConfig config;

    public:
        Tracker(const CameraConfig& cfg): config(cfg) {}
        float start(FilterResult& filter);  
};