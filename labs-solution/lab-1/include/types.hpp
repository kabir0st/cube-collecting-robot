#pragma once
#include <opencv2/opencv.hpp>
#include <mutex>
#include <atomic>

using namespace std;
using namespace cv;


// color tracking margins
static constexpr float HUE_MARGIN  = 10.f;  // hue is sensitive
static constexpr float SAT_MARGIN  = 40.f;  // sat can vary a lot in lighting
static constexpr float VAL_MARGIN  = 40.f;  // val changes with shadows/light

struct CameraConfig {
    int width;
    int height;
    int fps_target;
    CameraConfig(int w, int h, int f):
        width(w), height(h), fps_target(f) {}

};

struct FilterResult {
    Scalar dominant_color;
    Scalar upper1;
    Scalar lower1;
    Scalar upper2;
    Scalar lower2;
    bool ready = false;
    bool has_wrap_around = false;
};

struct TrackerResult {
    mutable mutex lock;
    atomic<bool> running{true};
    bool has_orientation = false;
    float orientation = 0;

    mutable mutex mutex_frame;
    int fps = 0;
    Mat frame;

    // Thread-safe
    void set_orientation(float value) {
        lock_guard<mutex> guard(lock);
        orientation = value;
        has_orientation = true;
    }

    void clear_orientation() {
        lock_guard<mutex> guard(lock);
        has_orientation = false;
        orientation = 0; //reset value
    }

    pair< float, int> get_orientation() const {
        lock_guard<mutex> guard(lock);
        return {orientation, fps};
    }

    bool is_orientation_available() const {
        lock_guard<mutex> guard(lock);
        return has_orientation;
    }

    void update_frame(const Mat& f, int frame_rate) {
        lock_guard<mutex> guard(mutex_frame);
        frame = f;
        fps = frame_rate;
        }

    Mat get_frame() const {
        lock_guard<mutex> guard(mutex_frame);
        return frame.clone();
    }

    void stop() {
        running.store(false);
    }
    bool is_running() const {
        return running.load();
    }
};
