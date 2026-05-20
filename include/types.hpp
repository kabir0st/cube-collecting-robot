#pragma once

#include <vector>
#include <array>
#include <atomic>
#include <mutex>
#include <opencv2/opencv.hpp>

// only keep 3 latest frames
constexpr std::size_t FRAME_COUNT = 3;

struct Frame {
    int id;
    std::vector<double> ang;
    std::vector<double> dist;

    Frame() {
        ang.reserve(360);
        dist.reserve(360);
    }
};


struct LidarData {
    mutable std::mutex lock;
    double sensor_noise;

    std::array<Frame, FRAME_COUNT> frames;
    std::atomic<int> buffer_id;
    LidarData(double noise = 5.0);
    void update_buffer_id();
    std::array<Frame, int( FRAME_COUNT - 1)> get_latest_snapshot() const;
};

struct LineSeg {
    float nx = 0.0f, ny = 0.0f, r = 0.0f;
    float ax = 0.0f, ay = 0.0f, bx = 0.0f, by = 0.0f;
};

// for map
struct LineFit
{
    double cx, cy;
    double dx, dy;
    double nx, ny;
};

struct Point {
    double x, y;
    Point(double x_ = 0.0, double y_ = 0.0) : x(x_), y(y_) {}
};


struct RansacParams {
    // perpendicular gate
    double inlier_thresh_mm = 25.0;
    // inner RANSAC trials
    int max_iters_per_line = 200;
    int inlier_points_req = 12;
    // %age limit that should be inlier
    double min_inlier_fraction = 0.015;
    // hard cap on walls extracted
    int max_lines = 16; 
    // extend segments past min/max inlier projection by this much
    // so endpoint slack covers gaps
    double endpoint_padding_mm = 50.0;
    // 0 -> std::random_device
    uint32_t seed = 0xC0FFEEu; 
};

struct Pos2D {
    double x;       // mm
    double y;       // mm
    double theta;   // rad, wrapped to (-pi, pi] by the EKF

    Pos2D() : x(0.0), y(0.0), theta(0.0) {}
    Pos2D(double x_, double y_, double theta_)
        : x(x_), y(y_), theta(theta_) {}
};

struct PosEstimate {
    Pos2D pos;
    // 3x3 cov matrix for x,y, theta
    float cov[9];
    float sigma_x;
    float sigma_y;
    float sigma_a;

    int itr;
    // number of lidar points 
    int inliers;
    bool converged;
};


// cox co-variance
struct Pose2DCov {
    double xx = 0.0;
    double xy = 0.0;
    double xt = 0.0;
    double yy = 0.0;
    double yt = 0.0;
    double tt = 0.0;
};

// ================= For tracking Voltage =================
const float SHUNT_OHMS = 0.01;
const float MAX_EXPECTED_AMPS = 3.2;

constexpr float LOW_BATTERY_LIMIT = 11.25f;   // ~3.75 V/cell, ~25% SoC
constexpr float BATTERY_PRESENT_MIN_V = 1.0f; // below this, treat rail as "no battery wired" and skip
constexpr int LOW_HITS_NEEDED = 5;   // require N consecutive readings (debounce sag)
constexpr auto CHECK_PERIOD = 3; // check every 3 seconds

// for vision tracking
// color tracking margins
static constexpr float HUE_MARGIN  = 10.f;  // hue is sensitive
static constexpr float SAT_MARGIN  = 40.f;  // sat can vary a lot in lighting
static constexpr float VAL_MARGIN  = 40.f;  // val changes with shadows/light

// ---- Target box geometry ------------------------------------------------
// Physical dimensions of the box we're picking up, in millimetres.
// The pinhole-size distance method matches BOX_HEIGHT_MM against the image
// bounding-box's *vertical* extent — that's the dimension that stays
// consistent regardless of how the box is rotated about the vertical axis
// (assuming the box sits upright on the floor).
static constexpr double BOX_WIDTH_MM  = 80.0;  // long horizontal side
static constexpr double BOX_LENGTH_MM = 80.0;  // short horizontal side
static constexpr double BOX_HEIGHT_MM = 40.0;  // vertical (used for distance)

struct CameraConfig {
    int width;
    int height;
    int fps_target;

    // Pinhole-camera optics for distance estimation.
    //
    // focal_length_px is camera-specific; calibrate once by holding the box
    // at a known distance and computing:
    //     f_px = (pixel_height_observed * distance_mm) / BOX_HEIGHT_MM
    // 650 matches the current rig (bbox.height=87.5 px at 297 mm with a
    // 40 mm box). Re-measure when swapping cameras.
    double focal_length_px = 650.0;

    // Distance/bearing thresholds for the capture state machine.
    // Capture entry: distance_mm <= capture_distance_mm AND
    //   |bearing_rad| <= capture_bearing_tol_rad, sustained for the
    //   capture-frame debounce. Lateral alignment matters at entry —
    //   the servo needs the box in the claw mouth, not off to a side.
    // Retention: once captured, the servo is mechanically holding the
    //   box, so vision only needs a loose "still close-ish" check.
    //   distance_mm > retention_distance_mm for the retention-frame
    //   debounce (or sustained no-detection) clears captured. No
    //   bearing check during retention — claw arms can partially
    //   occlude the box during drag and skew the centroid.
    double capture_distance_mm     = 120.0;   // grab-pose distance
    double capture_bearing_tol_rad = 0.12;    // ~7°
    double retention_distance_mm   = 150.0;   // matches claw reach

    // When true, the tracker pops an OpenCV window with the live frame
    // and a values HUD for tuning. Disable when running headless
    // (no DISPLAY / no X-forwarding over SSH).
    bool show_debug_view = true;

    // Near-field "grabbable" gate — used to set TrackerResult::grabbable.
    // Grabbable when the target bbox occupies most of the bottom region
    // of the frame:
    //   bottom_region = the lower `grab_bottom_region_ratio` of the image
    //                   (y in [(1 - r) * H, H], full image width).
    //   grabbable when  area(bbox ∩ bottom_region)
    //                 / area(bottom_region)  >=  grab_bottom_fill_ratio.
    // Independent of focal length / pinhole distance; the bbox simply
    // has to physically fill the part of the frame the claw will close
    // on. A high fill ratio implicitly demands horizontal coverage too,
    // so no separate lateral-centering gate is needed.
    double grab_bottom_region_ratio = 0.40;
    double grab_bottom_fill_ratio   = 0.50;

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

// Result of the camera-tracking worker (`loop_vision_tracking`).
struct TrackerResult {
    mutable std::mutex lock;
    std::atomic<bool> running{true};
    std::atomic<bool> captured{false};
    std::atomic<bool> lost{true};
    // Instantaneous near-field signal: true while the bbox fills most
    // of the bottom region of the frame, with brief hysteresis. Distinct
    // from `captured`, which is sticky across the drag phase. See
    // CameraConfig::grab_bottom_fill_ratio.
    std::atomic<bool> grabbable{false};

    bool is_captured()  const { return captured.load(); }
    bool is_lost()      const { return lost.load(); }
    bool is_grabbable() const { return grabbable.load(); }

    // Latest target measurement.
    bool   has_target  = false;
    double bearing_rad = 0.0;        // angle from optical axis, +left
    double distance_mm = 0.0;        // pinhole-size estimate (Method A)
    double area_px     = 0.0;        // contour area in pixels (confidence)

    mutable std::mutex mutex_frame;
    std::atomic<int> fps{0};
    cv::Mat frame;

    void set_target(double bearing,
                    double dist,
                    double area) {
        std::lock_guard<std::mutex> guard(lock);
        bearing_rad        = bearing;
        distance_mm        = dist;
        area_px            = area;
        has_target         = true;
    }

    void clear_target() {
        std::lock_guard<std::mutex> guard(lock);
        has_target        = false;
        bearing_rad       = 0.0;
        distance_mm       = 0.0;
        area_px           = 0.0;
    }

    struct Snapshot {
    bool   has_target;
    double bearing_rad;
    double distance_mm;          // pinhole-size estimate (Method A)
    double area_px;
    int    fps;
    bool   captured;
    bool   lost;
    bool   grabbable;
    };


    Snapshot get_snapshot() const {
        std::lock_guard<std::mutex> guard(lock);
        return { has_target, bearing_rad,
                 distance_mm,
                 area_px, fps,
                 captured.load(), lost.load(), grabbable.load() };
    }

    void update_frame(const cv::Mat &f, int frame_rate);
    cv::Mat get_frame() const;
    void stop() { running.store(false); }
    bool is_running() const { return running.load(); }
};