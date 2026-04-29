#include "calibration.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <vector>
#include <algorithm>

using namespace std;
using namespace cv;

CalibrateCamera::CalibrateCamera(const CameraConfig& cfg) : config(cfg) {}

Scalar CalibrateCamera::hsvToRgb(float h, float s, float v) {
    Mat hsv(1, 1, CV_8UC3, Scalar(h, s, v));
    Mat rgb;
    cvtColor(hsv, rgb, COLOR_HSV2RGB);
    return Scalar(rgb.at<Vec3b>(0,0)[0], 
                     rgb.at<Vec3b>(0,0)[1], 
                     rgb.at<Vec3b>(0,0)[2]);
}

void CalibrateCamera::display_color_in_terminal(const Scalar& hsv_color) {
    Scalar rgb = hsvToRgb(hsv_color[0], hsv_color[1], hsv_color[2]);
    
    cout << "\r\033[K";
    cout << "Dominant Color: ";
    
    cout << "\033[48;2;" << (int)rgb[0] << ";" << (int)rgb[1] << ";" << (int)rgb[2] << "m";
    cout << "      ";
    cout << "\033[0m ";
    
    cout << "HSV(" << (int)hsv_color[0] << ", " 
              << (int)hsv_color[1] << ", " << (int)hsv_color[2] << ") ";
    cout << "RGB(" << (int)rgb[0] << "," << (int)rgb[1] << "," << (int)rgb[2] << ")";
    cout << flush;
}



bool CalibrateCamera::create_filter_mask(FilterResult& result){
    float h_min = 180.f, s_min = 255.f, v_min = 255.f;
    float h_max = 0.f, s_max = 0.f, v_max = 0.f;
    vector<float> h_vals;
    h_vals.reserve(d_index_pixel_count);
    for (int i = 0; i < labels.rows; i++) {
        if (labels.at<int>(i) == dominant_idx) {
            float h = pixels.at<float>(i, 0);
            float s = pixels.at<float>(i, 1);
            float v = pixels.at<float>(i, 2);
            h_vals.push_back(h);
            h_min = min(h_min, h);
            s_min = min(s_min, s);
            v_min = min(v_min, v);
            h_max = max(h_max, h);
            s_max = max(s_max, s);
            v_max = max(v_max, v);
        }
    }

    const float sv_floor_ratio = 0.35f;
    const float min_non_black_sv = 10.f;
    const float min_hue_confidence = 40.f;
    const float red_edge_window = 25.f;

    const float dominant_s = centers.at<float>(dominant_idx, 1);
    const float dominant_v = centers.at<float>(dominant_idx, 2);

    auto compute_lower_sv = [&](float min_channel, float dominant_channel) {
        const float raw_lower = max(0.f, min_channel);
        if (dominant_channel <= min_non_black_sv) {
            return raw_lower;
        }
        const float dynamic_floor = max(min_non_black_sv, dominant_channel * sv_floor_ratio);
        return max(raw_lower, dynamic_floor);
    };

    const float s_lower = compute_lower_sv(s_min, dominant_s);
    const float v_lower = compute_lower_sv(v_min, dominant_v);
    const float s_upper = min(255.f, s_max);
    const float v_upper = min(255.f, v_max);

    const bool hue_is_reliable =
        dominant_s >= min_hue_confidence && dominant_v >= min_hue_confidence;
    const bool spans_red_edges =
        h_min <= red_edge_window && h_max >= (180.f - red_edge_window);

    if ((h_max - h_min) > 90.f && hue_is_reliable && spans_red_edges){
        sort(h_vals.begin(), h_vals.end());
        float max_gap = 0.f;
        int gap_idx = 1;
        for (int i = 1; i < (int)h_vals.size(); i++) {
            float gap = h_vals[i] - h_vals[i - 1];
            if (gap > max_gap) {
                max_gap = gap;
                gap_idx = i;
            }
        }

        float h_low_max = h_vals[gap_idx - 1];
        float h_high_min = h_vals[gap_idx];

        // High side: ~170 → 180
        result.lower1 = Scalar (max(0.f, h_high_min), s_lower, v_lower);
        result.upper1 =  Scalar (180.f, s_upper, v_upper);

        // Low side: 0 → ~8
        result.lower2 = Scalar (0.f, s_lower, v_lower);
        result.upper2 = Scalar (min(180.f, h_low_max), s_upper, v_upper);
        result.has_wrap_around = true;
    }
    else
    {
        result.lower1 = Scalar (max(0.f, h_min), s_lower, v_lower);
        result.upper1 = Scalar (min(180.f, h_max), s_upper, v_upper);
        result.has_wrap_around = false;

    }
    return true;
}

Scalar CalibrateCamera::get_dominant_color(const Mat& frame, const Rect& roi) {
    Mat roi_frame = frame(roi);
    cvtColor(roi_frame, hsv_frame, COLOR_BGR2HSV);

    pixels = hsv_frame.reshape(1, hsv_frame.rows * hsv_frame.cols);
    pixels.convertTo(pixels, CV_32F);
    // Mat labels, centers; -> moved definition to class
    TermCriteria criteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 10, 1.0);
    kmeans(pixels, k_neighbors, labels, criteria, 10, KMEANS_RANDOM_CENTERS, centers);

    vector<int> counts(k_neighbors, 0);
    for (int i = 0; i < labels.rows; i++) {
        counts[labels.at<int>(i)]++;
    }

    dominant_idx = max_element(counts.begin(), counts.end()) - counts.begin();
    d_index_pixel_count = counts[dominant_idx];
    return Scalar(
        centers.at<float>(dominant_idx, 0),
        centers.at<float>(dominant_idx, 1),
        centers.at<float>(dominant_idx, 2));
}
FilterResult CalibrateCamera::start_filter_calibration() {
    FilterResult result;
    // VideoCapture cap(0);
    VideoCapture cap(0, CAP_V4L2);
    if (!cap.isOpened())
    {
        throw runtime_error("Cannot Open camera.");
    }
    cap.set(CAP_PROP_FRAME_WIDTH, config.width);
    cap.set(CAP_PROP_FRAME_HEIGHT, config.height);
    cap.set(CAP_PROP_FPS, config.fps_target);
    int w = config.width / 2;
    int h = config.height / 2;
    int x = (config.width - w) / 2;
    int y = (config.height - h) / 2;
    const Rect roi = Rect(x, y, w, h);
    Mat frame;
    Scalar dominant_color;
    while (true)
    {
        cap >> frame;
        if (frame.empty()) break;
        dominant_color = get_dominant_color(frame, roi);
        display_color_in_terminal(dominant_color);
        
        rectangle(frame, 
                    roi,
                    Scalar(0, 255, 0), 2);
        imshow("Calibration Setup", frame);
        char key = waitKey(1);
        if (key == 'q') {
            cout << "\nQuitting without selecting color" << endl;
            break;
        }
        else if (key == 'c') {
            cout << "\n\nColor captured!" << endl;
            cout << "Selected HSV: (" << (int)dominant_color[0] << ", " 
                 << (int)dominant_color[1] << ", " << (int)dominant_color[2] << ")" << endl;
            result.ready = create_filter_mask(result);
            break;
        }
    }
    destroyAllWindows();
    result.dominant_color = dominant_color;
    return result;
}
