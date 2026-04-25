#include "types.hpp"
#include "calibration.hpp"
#include <stdexcept>
#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
    try {
        CameraConfig config(640, 480, 120);
        CalibrateCamera calibration(config);
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
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}