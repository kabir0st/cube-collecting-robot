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


using namespace std;
using namespace cv;

TrackerResult tracking_result;

void main_logic() {
    // CameraConfig config(640, 480, 12);
    CameraConfig config(320, 240, 12);
    CalibrateCamera calibration(config);
    TrackerClass tracker = TrackerClass(config);
    auto filter = calibration.start_filter_calibration();
    if (!filter.ready) {
        throw runtime_error("Cannot get filter ready.");
    }
    cout << "Dominant HSV: " << filter.dominant_color << endl;
    cout << "Upperbound HSV: " << filter.upper1 << endl;
    cout << "Lowerbound HSV: " << filter.lower1 << endl;
    if (filter.has_wrap_around) {
        cout << "WrapUpper HSV: " << filter.upper2 << endl;
        cout << "Wraplower HSV: " << filter.lower2 << endl;
    }
    cout << "Starting tracking . . ." << endl;
    exception_ptr tracker_error;
    thread tracking_thread([&tracker, &filter, &tracker_error]() {
        try {
            tracker.start(filter, tracking_result);
        } catch (...) {
            tracker_error = current_exception();
            tracking_result.stop();
        }
    });

    namedWindow("Live", WINDOW_AUTOSIZE);
    printf("\n\n");
    while (tracking_result.is_running())
    {
        Mat frame = tracking_result.get_frame();
        if (!frame.empty()) {
            imshow("Live", frame);
        }
        const int key = waitKey(1);
        if (key == 'q') {
            tracking_result.stop();
            break;
        }
        if (frame.empty()) {
            this_thread::sleep_for(chrono::milliseconds(5));
        }
        auto result = tracking_result.get_orientation(); 
        if (tracking_result.is_orientation_available()) { 
            cout << "\nOrientation: " << result.first << " FPS: " << result.second;
        } else {
            cout << "\nOrientation: none FPS: " << result.second;
        }
        flush(cout);
        this_thread::sleep_for(chrono::milliseconds(10));
    }
    destroyAllWindows();

    if (tracking_thread.joinable()) {
        tracking_thread.join();
    }
    if (tracker_error) {
        rethrow_exception(tracker_error);
    }
}

int main() {
    try {
        main_logic();
    } catch (const exception &e) {
        cerr << "Error: " << e.what() << endl;
        return -1;
    }
    return 0;
}
