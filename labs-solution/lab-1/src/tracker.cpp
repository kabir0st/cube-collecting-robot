// #include "types.hpp"
// #include "tracker.hpp"
// #include <stdexcept>
// #include <iostream>
// #include <opencv2/opencv.hpp>
// #include <stdexcept>
// #include <vector>


// float Tracker::start(FilterResult& filter){
//     cv::VideoCapture cap(0, cv::CAP_V4L2);
//     if (!cap.isOpened()) {
//         throw std::runtime_error("Cannot open camera.\n");
//     }
//     cap.set(cv::CAP_PROP_FRAME_WIDTH, config.width);
//     cap.set(cv::CAP_PROP_FRAME_HEIGHT, config.height);
//     cap.set(cv::CAP_PROP_FPS, 12);
//     cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

//     std::vector<std::vector<cv::Point>> contours;
//     cv::Mat frame, hsv;
//     while (1){
//         cap >> frame;
//         if (frame.empty())
//             continue;
//         cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        
//         cv::Mat mask1, mask2;
//         cv::inRange(hsv_frame, lower1, upper1, mask1);
//         cv::inRange(hsv_frame, lower2, upper2, mask2);
//         cv::bitwise_or(mask1, mask2, mask);
//     }
// }