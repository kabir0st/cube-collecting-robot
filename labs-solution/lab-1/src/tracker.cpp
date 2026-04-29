#include "types.hpp"
#include "tracker.hpp"
#include <stdexcept>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <vector>
#include <cfloat>
#include <chrono>

using namespace std;
using namespace cv;

void TrackerClass::start(FilterResult& filter, TrackerResult &result){
    // VideoCapture cap(0);
    VideoCapture cap(0, CAP_V4L2);
    if (!cap.isOpened())
    {
        throw runtime_error("Cannot open camera.\n");
    }
    cap.set(CAP_PROP_FRAME_WIDTH, config.width);
    cap.set(CAP_PROP_FRAME_HEIGHT, config.height);
    cap.set(CAP_PROP_FPS, config.fps_target);
    cap.set(CAP_PROP_BUFFERSIZE, 1);

    vector<vector<Point>> contours;
    Mat frame, hsv_frame;
    // Mat open_kernel = getStructuringElement(MORPH_RECT, Size(7, 7));
    Mat close_kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    bool has_last_position = false;
    Point2f last_center;
    float cx=0, cy=0;
    result.clear_orientation();

    auto hz_last_time = chrono::steady_clock::now();
    int hz_frame_count = 0;
    
    while (result.is_running()) {
        cap >> frame;
        if (frame.empty()) {
            continue;
        }
        cvtColor(frame, hsv_frame, COLOR_BGR2HSV);
        Mat mask, wrap_mask;

        if (filter.has_wrap_around) {
            inRange(
                hsv_frame, filter.lower1, filter.upper1, mask);
            inRange(
                hsv_frame, filter.lower2, filter.upper2, wrap_mask);
            bitwise_or(mask, wrap_mask, mask);
        } else {
            inRange(
                hsv_frame, filter.lower1, filter.upper1, mask);
        }
        // erode(mask, mask, kernel);
        // dilate(mask, mask, kernel);c
        // morphologyEx(mask, mask, MORPH_OPEN, open_kernel);
        morphologyEx(mask, mask, MORPH_CLOSE, close_kernel);

        result.clear_orientation();
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        if (!contours.empty()) {
            int selected = -1;
            double selected_score = -DBL_MAX;
            Point2f selected_center;
            for (int i = 0; i < (int)contours.size(); i++)
            {
                double area = contourArea(contours[i]);
                if (area <= 700) {
                    continue;
                }
                Moments M = moments(contours[i]);
                if (M.m00 == 0) {
                    continue;
                }
                Point2f center((float)(M.m10 / M.m00), (float)(M.m01 / M.m00));
                double score = area;
                if (has_last_position) {
                    const double distance = norm(center - last_center);
                    score -= (8.0 * distance);
                }
                if (score > selected_score) {
                    selected_score = score;
                    selected = i;
                    selected_center = center;
                }
            }
            if (selected != -1){
                cx = selected_center.x;
                cy = selected_center.y;
                circle(frame, Point(cx, cy), 5, Scalar(0, 255, 0), -1);
                Rect bbox = boundingRect(contours[selected]);
                rectangle(frame, bbox, Scalar(0, 0, 255), 2);
                has_last_position = true;
                last_center = selected_center;
                result.set_orientation(config.width/2 - cx);
            }
        }
        
        hz_frame_count++;
        auto now = chrono::steady_clock::now();
        double elapsed = chrono::duration<double>(now - hz_last_time).count();
        result.update_frame(frame, (int)(hz_frame_count / elapsed));
        if (elapsed >= 1.0) {
            hz_frame_count = 0;
            hz_last_time = now;
        }
    }
    result.stop();
}
