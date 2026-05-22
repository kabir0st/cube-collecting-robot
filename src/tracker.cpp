#include <tracker.hpp>

#include <algorithm>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include <dashboard.hpp>
#include <global.hpp>
#include <loggers.hpp>
#include <types.hpp>

namespace {

    // Pinhole-size distance.
    //   distance_mm = f_px * BOX_HEIGHT_MM / box_height_observed_px
    //
    // We match the physical height (BOX_HEIGHT_MM) against the image
    // bounding-box's vertical extent. That ratio is invariant to rotation
    // about the vertical axis, so the estimate stays consistent however
    // the box is oriented on the floor — unlike matching against width,
    // which depends on whether the box is showing its long or short side.
    // Returns -1 if the value can't be computed.
    inline double pinhole_distance_mm(const cv::Rect& bbox,
                                      const CameraConfig& cfg) {
        if (cfg.focal_length_px <= 0.0 || BOX_HEIGHT_MM <= 0.0) {
            return -1.0;
        }
        if (bbox.height <= 0) return -1.0;
        return cfg.focal_length_px * BOX_HEIGHT_MM
             / static_cast<double>(bbox.height);
    }

    // Bearing of the target relative to the camera's optical axis.
    // +left, -right, in radians.
    inline double bearing_rad(double cx_target, const CameraConfig& cfg) {
        const double cx_image = 0.5 * static_cast<double>(cfg.width);
        return std::atan2(cx_image - cx_target, cfg.focal_length_px);
    }

}  // namespace

void loop_vision_tracking(Dashboard& dash,
                          FilterResult filter,
                          CameraConfig cfg,
                          TrackerResult& result) {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        LOGE("Vision") << "Cannot open camera 0";
        return;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  cfg.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, cfg.height);
    cap.set(cv::CAP_PROP_FPS,          cfg.fps_target);
    cap.set(cv::CAP_PROP_BUFFERSIZE,   1);

    cv::Mat close_kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(5, 5));

    cv::Mat frame, hsv_frame, mask, wrap_mask;
    std::vector<std::vector<cv::Point>> contours;

    // Continuity bias: prefer detections close to the last accepted one
    // (kills momentary swaps to a smaller distant blob with similar hue).
    bool has_last_position = false;
    cv::Point2f last_center;

    // Capture-state hysteresis counters (see TrackerResult::captured/lost).
    // All thresholds are ~0.66s at 15 FPS — long enough to ignore single-
    // frame HSV blinks, short enough that the state machine reacts quickly.
    constexpr int CAPTURE_FRAMES   = 10;
    constexpr int RETENTION_FRAMES = 10;
    constexpr int LOST_FRAMES      = 10;
    // Grabbable hysteresis: shorter so the servo gets the signal quickly
    // once the box is in range, but still long enough to ride out a one-
    // or two-frame HSV blink (~0.53 s at 15 fps).
    constexpr int GRAB_FRAMES      = 8;
    constexpr int RELEASE_FRAMES   = 8;
    // Counters saturate at their thresholds (see clamp_inc) so a long
    // detection streak / dropout doesn't accumulate boundless integer
    // values. unsigned makes the saturation intent explicit and removes
    // any signed-overflow UB question.
    unsigned in_capture_counter       = 0;
    unsigned out_of_retention_counter = 0;
    unsigned no_detection_counter     = 0;
    unsigned in_grab_counter          = 0;
    unsigned out_of_grab_counter      = 0;
    auto clamp_inc = [](unsigned& c, unsigned cap) {
        if (c < cap) ++c;
    };

    // Local mirror of the published flags. Single-source-of-truth for the
    // debounce state machines below; committed in one critical section per
    // frame via result.commit_flags() so readers never see torn triples.
    bool flag_captured  = false;
    bool flag_lost      = true;
    bool flag_grabbable = false;

    auto hz_last_time = std::chrono::steady_clock::now();
    int  hz_frame_count = 0;
    int  current_fps    = 0;

    LOG("Vision") << "Vision tracking started ("
                  << cfg.width << "x" << cfg.height
                  << " @ " << cfg.fps_target << " target fps, "
                  << "f_px=" << cfg.focal_length_px
                  << ", box H=" << BOX_HEIGHT_MM << " mm)";

    while (!app::shutdown && result.is_running()) {
        cap >> frame;
        if (frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);
        if (filter.has_wrap_around) {
            cv::inRange(hsv_frame, filter.lower1, filter.upper1, mask);
            cv::inRange(hsv_frame, filter.lower2, filter.upper2, wrap_mask);
            cv::bitwise_or(mask, wrap_mask, mask);
        } else {
            cv::inRange(hsv_frame, filter.lower1, filter.upper1, mask);
        }
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, close_kernel);

        cv::findContours(mask, contours,
                         cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Score = area, penalized by distance from the last detection.
        int          selected       = -1;
        double       selected_score = -DBL_MAX;
        cv::Point2f  selected_center;
        cv::Rect     selected_bbox;
        double       selected_area  = 0.0;

        for (size_t i = 0; i < contours.size(); ++i) {
            const double area = cv::contourArea(contours[i]);
            if (area <= 500.0) continue;
            const cv::Moments M = cv::moments(contours[i]);
            if (M.m00 == 0.0) continue;
            const cv::Point2f center(
                static_cast<float>(M.m10 / M.m00),
                static_cast<float>(M.m01 / M.m00));
            double score = area;
            if (has_last_position) {
                score -= 8.0 * cv::norm(center - last_center);
            }
            if (score > selected_score) {
                selected_score  = score;
                selected        = static_cast<int>(i);
                selected_center = center;
                selected_bbox   = cv::boundingRect(contours[i]);
                selected_area   = area;
            }
        }

        // Bottom-region "grabbable" zone — depends only on cfg, so
        // compute once per frame regardless of detection. Drawn in the
        // debug overlay below so the gate is tunable by eye.
        const int top_y = static_cast<int>(
            cfg.height * (1.0 - cfg.grab_bottom_region_ratio));
        const cv::Rect bottom_region(0, top_y,
                                     cfg.width, cfg.height - top_y);

        // Compute per-frame gates. When no blob was selected, all gates
        // are false — the state machines below then treat the frame as a
        // pure "out-of-gate" tick, which naturally drives release.
        const bool detected = (selected != -1);
        bool   grab_gate         = false;
        bool   in_capture_range  = false;
        bool   in_retention_range = false;
        double dist_mm  = -1.0;
        double bearing = 0.0;

        if (detected) {
            dist_mm = pinhole_distance_mm(selected_bbox, cfg);
            bearing = bearing_rad(selected_center.x, cfg);

            const double region_area  =
                static_cast<double>(bottom_region.area());
            const double overlap_area =
                static_cast<double>((selected_bbox & bottom_region).area());
            const double bottom_fill  =
                (region_area > 0.0) ? overlap_area / region_area : 0.0;
            grab_gate = bottom_fill >= cfg.grab_bottom_fill_ratio;

            in_capture_range = dist_mm > 0.0
                && dist_mm <= cfg.capture_distance_mm
                && std::abs(bearing) <= cfg.capture_bearing_tol_rad;
            in_retention_range = dist_mm > 0.0
                && dist_mm <= cfg.retention_distance_mm;

            cv::circle(frame, cv::Point(selected_center), 5,
                       cv::Scalar(0, 255, 0), -1);
            cv::rectangle(frame, selected_bbox, cv::Scalar(0, 0, 255), 2);

            result.set_target(bearing, dist_mm, selected_area);
            has_last_position = true;
            last_center       = selected_center;
        } else {
            result.clear_target();
            has_last_position = false;
        }

        cv::rectangle(frame, bottom_region, cv::Scalar(0, 255, 255), 1);

        // -------- Lost state machine --------
        if (detected) {
            no_detection_counter = 0;
            if (flag_lost) {
                flag_lost = false;
                LOG("Vision") << "Target re-acquired";
            }
        } else {
            clamp_inc(no_detection_counter,
                      static_cast<unsigned>(LOST_FRAMES));
            if (no_detection_counter >= static_cast<unsigned>(LOST_FRAMES)
                && !flag_lost) {
                flag_lost = true;
                LOG("Vision") << "LOST (no detection)";
            }
        }

        // -------- Capture / retention state machine --------
        // No-detection naturally feeds in_*_range=false, so the same
        // logic handles dropout-driven release.
        if (!flag_captured) {
            if (in_capture_range) {
                clamp_inc(in_capture_counter,
                          static_cast<unsigned>(CAPTURE_FRAMES));
                if (in_capture_counter >= static_cast<unsigned>(CAPTURE_FRAMES)) {
                    flag_captured = true;
                    out_of_retention_counter = 0;
                    LOG("Vision") << "CAPTURED";
                }
            } else {
                in_capture_counter = 0;
            }
        } else {
            if (in_retention_range) {
                out_of_retention_counter = 0;
            } else {
                clamp_inc(out_of_retention_counter,
                          static_cast<unsigned>(RETENTION_FRAMES));
                if (out_of_retention_counter >=
                        static_cast<unsigned>(RETENTION_FRAMES)) {
                    flag_captured = false;
                    in_capture_counter = 0;
                    LOG("Vision") << "Released";
                }
            }
        }

        // -------- Grabbable state machine --------
        // Instantaneous near-field + lateral gate, debounced. Distinct
        // from `captured`: this is "should the servo close right now?".
        if (grab_gate) {
            out_of_grab_counter = 0;
            if (!flag_grabbable) {
                clamp_inc(in_grab_counter,
                          static_cast<unsigned>(GRAB_FRAMES));
                if (in_grab_counter >= static_cast<unsigned>(GRAB_FRAMES)) {
                    flag_grabbable = true;
                    LOG("Vision") << "GRABBABLE";
                }
            }
        } else {
            in_grab_counter = 0;
            if (flag_grabbable) {
                clamp_inc(out_of_grab_counter,
                          static_cast<unsigned>(RELEASE_FRAMES));
                if (out_of_grab_counter >=
                        static_cast<unsigned>(RELEASE_FRAMES)) {
                    flag_grabbable = false;
                    LOG("Vision") << "Not grabbable";
                }
            }
        }

        // Publish all three flags atomically so readers never see a torn
        // triple. set_target/clear_target above already took the lock for
        // bearing/dist; this is a separate critical section because the
        // state-machine logic between then and now consumes the previous
        // flag values.
        result.commit_flags(flag_captured, flag_lost, flag_grabbable);

        // FPS estimator (averages over ~1 s windows).
        ++hz_frame_count;
        const auto now = std::chrono::steady_clock::now();
        const double elapsed =
            std::chrono::duration<double>(now - hz_last_time).count();
        if (elapsed >= 1.0) {
            current_fps   = static_cast<int>(hz_frame_count / elapsed);
            hz_frame_count = 0;
            hz_last_time   = now;
        }

        if (cfg.show_debug_view && std::getenv("DISPLAY")) {
            cv::imshow("Vision Tracker", frame);
            cv::waitKey(1);
        }

        result.update_frame(frame, current_fps);

        // Dashboard publish.
        const TrackerResult::Snapshot snap = result.get_snapshot();
        dash.set("Vision", "fps",    static_cast<int64_t>(snap.fps));
        dash.set("Vision", "target",
                 std::string(snap.has_target ? "yes" : "no"));
        dash.set("Vision", "captured",
                 std::string(snap.captured ? "yes" : "no"));
        dash.set("Vision", "lost",
                 std::string(snap.lost ? "yes" : "no"));
        dash.set("Vision", "grabbable",
                 std::string(snap.grabbable ? "yes" : "no"));
        if (snap.has_target) {
            dash.set("Vision", "bearing_deg",
                     snap.bearing_rad * 180.0 / M_PI);
            dash.set("Vision", "dist_mm", snap.distance_mm);
            dash.set("Vision", "area_px", snap.area_px);
        }
    }

    if (cfg.show_debug_view && std::getenv("DISPLAY")) {
        cv::destroyWindow("Vision Tracker");
    }
    LOG("Vision") << "Vision tracking stopped";
    result.stop();
}
