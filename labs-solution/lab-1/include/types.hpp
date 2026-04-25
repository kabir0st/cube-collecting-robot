#pragma once
#include <opencv2/opencv.hpp>
#include <mutex>
#include <atomic>

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
    cv::Scalar dominant_color;
    cv::Scalar upper1;
    cv::Scalar lower1;
    cv::Scalar upper2;
    cv::Scalar lower2;
    bool ready = false;
    bool has_wrap_around = false;
};

struct TrackerResult {
    mutable std::mutex lock;
    std::atomic<bool> running{true};
    bool has_orientation = false;
    float orientation = 0;

    mutable std::mutex mutex_frame;
    int fps = 0;
    cv::Mat frame;

    // Thread-safe
    void set_orientation(float value) {
        std::lock_guard<std::mutex> guard(lock);
        orientation = value;
        has_orientation = true;
    }

    void clear_orientation() {
        std::lock_guard<std::mutex> guard(lock);
        has_orientation = false;
        orientation = 0; //reset value
    }

    float get_orientation() const {
        std::lock_guard<std::mutex> guard(lock);
        return orientation;
    }

    bool is_orientation_available() const {
        std::lock_guard<std::mutex> guard(lock);
        return has_orientation;
    }

    void update_frame(const cv::Mat& f) {
        std::lock_guard<std::mutex> guard(mutex_frame);
        frame = f;
    }

    cv::Mat get_frame() const {
        std::lock_guard<std::mutex> guard(mutex_frame);
        return frame.clone();
    }

    void stop() {
        running.store(false);
    }
    bool is_running() const {
        return running.load();
    }
};
