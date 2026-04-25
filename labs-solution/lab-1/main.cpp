#include "types.hpp"
#include "calibration.hpp"
#include "tracker.hpp"
#include <stdexcept>
#include <iostream>
#include <opencv2/opencv.hpp>

// will be handled using threads in future
void start_tracking(Tracker& tracker, FilterResult& filter){
    tracker.start(filter);
}

int main() {
    try {
        CameraConfig config(640, 480, 120);
        CalibrateCamera calibration(config);
        Tracker tracker = Tracker(config);
        auto filter = calibration.start_filter_calibration();
        std::cout << "filter is ready? " << (filter.ready ? "true" : "false") << std::endl;
        if (filter.ready) {
            std::cout << "Dominant HSV: " << filter.dominant_color << std::endl;
            std::cout << "Upperbound HSV: " << filter.upper1 << std::endl;
            std::cout << "Lowerbound HSV: " << filter.lower1 << std::endl;
            if (filter.has_wrap_around) {
                std::cout << "WrapUpper HSV: " << filter.upper2 << std::endl;
                std::cout << "Wraplower HSV: " << filter.lower2 << std::endl;
            }
            cv::waitKey(5000);
            std::cout << "Starting tracking . . ." << std::endl;
            // thread out to start tracking here
            start_tracking(tracker, filter);
        }
        else
        {
            std::cout << "Couldn't get filter ready.";
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}