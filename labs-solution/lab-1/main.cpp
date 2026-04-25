#include "types.hpp"
#include "calibration.hpp"
#include "tracker.hpp"
#include <stdexcept>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <exception>

TrackerResult tracking_result = TrackerResult();

void main_logic() {
    // CameraConfig config(640, 480, 12);
    CameraConfig config(320, 240, 12);
    CalibrateCamera calibration(config);
    Tracker tracker = Tracker(config);
    auto filter = calibration.start_filter_calibration();
    if (!filter.ready) {
        throw std::runtime_error("Cannot get filter ready.");
    }
    std::cout << "Dominant HSV: " << filter.dominant_color << std::endl;
    std::cout << "Upperbound HSV: " << filter.upper1 << std::endl;
    std::cout << "Lowerbound HSV: " << filter.lower1 << std::endl;
    if (filter.has_wrap_around) {
        std::cout << "WrapUpper HSV: " << filter.upper2 << std::endl;
        std::cout << "Wraplower HSV: " << filter.lower2 << std::endl;
    }
    std::cout << "Starting tracking . . ." << std::endl;
    std::exception_ptr tracker_error;
    std::thread tracking_thread([&tracker, &filter, &tracker_error]() {
        try {
            tracker.start(filter, tracking_result);
        } catch (...) {
            tracker_error = std::current_exception();
            tracking_result.stop();
        }
    });

    cv::namedWindow("Live", cv::WINDOW_AUTOSIZE);
    printf("\n\n");
    while (tracking_result.is_running())
    {
        cv::Mat frame = tracking_result.get_frame();
        if (!frame.empty()) {
            cv::imshow("Live", frame);
        }
        const int key = cv::waitKey(1);
        if (key == 'q') {
            tracking_result.stop();
            break;
        }
        if (frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        std::cout << "\rOrientation: " << tracking_result.get_orientation();
        std::cout.flush();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    cv::destroyAllWindows();

    if (tracking_thread.joinable()) {
        tracking_thread.join();
    }
    if (tracker_error) {
        std::rethrow_exception(tracker_error);
    }
}

int main() {
    try {
        main_logic();
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}
