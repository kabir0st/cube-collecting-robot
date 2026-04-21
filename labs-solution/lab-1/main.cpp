#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <thread>

using namespace cv;
using namespace std;

int get_center_distance(int debug=0) {
  VideoCapture cap(0, cv::CAP_V4L2);
  if (!cap.isOpened()){
    cerr << "Error: Unable to open camera" << endl;
    return 0;
  }
  cap.set(CAP_PROP_FRAME_WIDTH, 640);
  cap.set(CAP_PROP_FRAME_HEIGHT, 480);

  // White Hue, Saturation, Brightness
  Scalar lower = Scalar(0, 0, 200);
  Scalar upper = Scalar(180, 20, 255);

  // Red
  // Scalar lower = Scalar(0, 100, 100);
  // Scalar upper = Scalar(10, 255, 255);

  // Black
  // Scalar lower = Scalar(0, 0, 0);
  // Scalar upper = Scalar(180, 255, 50);

  // testing different shapes
  // Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
  Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));

  vector<vector<Point>> contours;
  Mat frame, hsv, mask;

  // fps counter
  auto fps_start = chrono::steady_clock::now();
  int frames, cx, cy, fps, obj_orientation, obj_tracking = 0;
  while (1)
  {
    auto loop_start = chrono::steady_clock::now();
    cap >> frame;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    inRange(hsv, lower, upper, mask);
    erode(mask, mask, kernel);
    dilate(mask, mask, kernel);
    // Find contours on cleaned mask
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    obj_tracking = 0;
    if (!contours.empty())
    {
      int largest = 0;
      double max_area = 0;
      for (int i = 0; i < (int)contours.size(); i++){
        double area = contourArea(contours[i]);
        if (area > max_area && area > 2500){
          max_area = area;
          largest = i;
        }
      }

    if (largest) {
        Moments M = moments(contours[largest]);
        cx = (int)(M.m10 / M.m00);
        cy = (int)(M.m01 / M.m00);
        // Draw center point and bounding box
        circle(frame, Point(cx, cy), 5, Scalar(0, 255, 0), -1);
        Rect bbox = boundingRect(contours[largest]);
        rectangle(frame, bbox, Scalar(0, 0, 255), 2);
        obj_tracking = 1;
        obj_orientation = 320 - cx;
      }
    }

    // doing some optimization
    auto now = chrono::steady_clock::now();
    if (debug) {
      imshow("Live", frame);
      imshow("Masked", mask);
      // fps counter
      frames++;
      double fps_elapsed = chrono::duration<double>(now - fps_start).count();
      if (fps_elapsed >= 1.0) {
        fps = frames / fps_elapsed;
        frames = 0;
        fps_start = now;
      }
    printf("\rFPS: %d | Center: %d, %d | Orientation: %s   ",
    fps, cx, cy, obj_tracking ? to_string(obj_orientation).c_str() : "None");
    fflush(stdout);
    }

    // cap fps
    double elapsed_ms = chrono::duration<double, milli>(now - loop_start).count();
    double target_ms = 1000.0 / 12;
    if (elapsed_ms < target_ms) {
      this_thread::sleep_for(chrono::milliseconds((int)(target_ms - elapsed_ms)));
    }
    if (waitKey(1) == 'q') break;
  }
  return 0;
}

int main() {
  get_center_distance(1);
  return 0;
}