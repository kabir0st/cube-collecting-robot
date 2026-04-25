#include "types.hpp"
#include "tracker.hpp"
#include <stdexcept>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <vector>


float Tracker::start(FilterResult& filter){
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
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    
    float cx, cy, fps, obj_tracking, obj_orientation = 0;
    std::cout << "\n\n";
    while (1)
    {
        cap >> frame;
        if (frame.empty())
            continue;
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
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

        int largest = -1;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (!contours.empty()) {
            double max_area = 0;
            largest = -1;
            for (int i = 0; i < contours.size(); i++) {
                double area = cv::contourArea(contours[i]);
                if ((area > max_area)) {
                    max_area = area;
                    largest = i;
                }
            }
            if (largest != -1){
                cv::Moments M = moments(contours[largest]);  // What you're using now
                if (M.m00 !=0) {
                    cx = M.m10 / M.m00;
                    cy = M.m01 / M.m00;
                    cv::circle(frame, cv::Point(cx, cy), 5, cv::Scalar(0, 255, 0), -1);
                    cv::Rect bbox = boundingRect(contours[largest]);
                    rectangle(frame, bbox, cv::Scalar(0, 0, 255), 2);    
                }
            }
        }
    
        imshow("Live", frame);
        // printf("\rFPS: %d | Center: %d, %d | Orientation: %s   ",
        // fps, cx, cy, obj_tracking ? std::to_string(obj_orientation).c_str() : "None");
        // fflush(stdout);
        if (cv::waitKey(1) == 'q' || cv::waitKey(1) == 27) {
        std::cout << "\nExiting...\n";
        break;
        }
    }
    cv::destroyAllWindows();
    return obj_orientation;
}