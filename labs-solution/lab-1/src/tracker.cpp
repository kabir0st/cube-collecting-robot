#include "types.hpp"
#include "tracker.hpp"
#include <stdexcept>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <vector>
#include <cfloat>
#include <chrono>


void Tracker::start(FilterResult& filter, TrackerResult &result){
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        throw std::runtime_error("Cannot open camera.\n");
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, config.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, config.height);
    cap.set(cv::CAP_PROP_FPS, 12);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    std::vector<std::vector<cv::Point>> contours;
    cv::Mat frame, hsv_frame;
    // cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    bool has_last_position = false;
    cv::Point2f last_center;
    float cx=0, cy=0;
    result.clear_orientation();

    while (result.is_running()) {
        cap >> frame;
        if (frame.empty()) {
            continue;
        }
        cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
        cv::Mat mask, wrap_mask;

        if (filter.has_wrap_around) {
            cv::inRange(
                hsv_frame, filter.lower1, filter.upper1, mask);
            cv::inRange(
                hsv_frame, filter.lower2, filter.upper2, wrap_mask);
            cv::bitwise_or(mask, wrap_mask, mask);
        } else {
            cv::inRange(
                hsv_frame, filter.lower1, filter.upper1, mask);
        }
        // cv::erode(mask, mask, kernel);
        // cv::dilate(mask, mask, kernel);c
        // cv::morphologyEx(mask, mask, cv::MORPH_OPEN, open_kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, close_kernel);

        result.clear_orientation();
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (!contours.empty()) {
            int selected = -1;
            double selected_score = -DBL_MAX;
            cv::Point2f selected_center;
            for (int i = 0; i < (int)contours.size(); i++)
            {
                double area = cv::contourArea(contours[i]);
                if (area <= 700) {
                    continue;
                }
                cv::Moments M = moments(contours[i]);
                if (M.m00 == 0) {
                    continue;
                }
                cv::Point2f center((float)(M.m10 / M.m00), (float)(M.m01 / M.m00));
                double score = area;
                if (has_last_position) {
                    const double distance = cv::norm(center - last_center);
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
                cv::circle(frame, cv::Point(cx, cy), 5, cv::Scalar(0, 255, 0), -1);
                cv::Rect bbox = boundingRect(contours[selected]);
                rectangle(frame, bbox, cv::Scalar(0, 0, 255), 2);
                has_last_position = true;
                last_center = selected_center;
                result.set_orientation(config.width/2 - cx);
            }
        }
        result.update_frame(frame);
    }
    result.stop();
}
