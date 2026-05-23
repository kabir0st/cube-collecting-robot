#pragma once

#include <vector>
#include <array>
#include <atomic>
#include <limits>
#include <mutex>
#include <opencv2/opencv.hpp>

// Circular buffer of lidar frames. Sized to give the boot-time map
// builder (find_walls / get_lidar_sensor_noise) enough data to extract
// stable walls via RANSAC. Runtime cox uses only the newest frame, so
// the buffer depth costs ~290 KB of RAM and a one-shot snapshot copy
// per boot — both negligible.
constexpr std::size_t FRAME_COUNT = 50;

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
    // sensor_noise is written once at boot (from loop_localization, after
    // a settling sleep) and read by cox / find_walls / telemetry from
    // worker threads. Atomic so the read/write is well-defined without
    // making the lidar buffer lock visible at every reader.
    std::atomic<double> sensor_noise;

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
    // Drop extracted segments shorter than this. 0 = no filter. In a
    // cluttered room the back edge of a chair / desk is short and produces
    // false "walls"; setting this to ~800 mm keeps only real walls.
    double min_segment_len_mm = 0.0;
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
    Pos2D pos{};
    // 3x3 cov matrix for x,y, theta
    float cov[9]{};
    float sigma_x = 0.0f;
    float sigma_y = 0.0f;
    float sigma_a = 0.0f;

    int itr = 0;
    // number of lidar points
    int inliers = 0;
    bool converged = false;

    // Information-matrix condition number λ_max / λ_min of A in the cox
    // normal equations. Large values (e.g. > 1e5) mean the scan is
    // geometrically degenerate (long corridor, one wall visible) and the
    // reported covariance is meaningless in the unobserved direction.
    // Set to +inf if the eigensolve fails or n_in < 6 (no fit at all).
    float cond_number = std::numeric_limits<float>::infinity();
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

// ---- Target box geometry ---------------------------------------
static constexpr double BOX_WIDTH_MM  = 80.0;  // long horizontal side
static constexpr double BOX_LENGTH_MM = 80.0;  // short horizontal side
static constexpr double BOX_HEIGHT_MM = 40.0;  // vertical (used for distance)

struct CameraConfig {
    int width;
    int height;
    int fps_target;
    double focal_length_px = 650.0;

    double capture_distance_mm     = 120.0;   // grab-pose distance
    double capture_bearing_tol_rad = 0.12;    // ~7°
    double retention_distance_mm   = 150.0;   // matches claw reach

    bool show_debug_view = true;

    double grab_bottom_region_ratio = 0.40;
    double grab_bottom_fill_ratio   = 0.50;
    // Max axis-misalignment of the detected box (from cv::minAreaRect),
    // in degrees. Squares are 90°-symmetric so the residual is in [0,45].
    double grab_angle_tol_deg       = 5.0;

    // Detections whose centroid falls inside the top fraction of the frame
    // are rejected outright (typical use: ignore pink things on walls/lab
    // posters above the floor plane).
    double detect_top_exclude_ratio = 0.25;

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

    // captured/lost/grabbable USED to be std::atomic<bool> set independently
    // across ~50 lines of tracker.cpp. The reader (StateMachine::step) takes
    // them all in one snapshot under `lock`, so previously a reader could
    // observe e.g. captured=true && lost=true mid-update. Now all three are
    // plain bools protected by `lock`, and the writer commits them in one
    // critical section via commit_flags() at end-of-frame.
    bool captured  = false;
    bool lost      = true;
    bool grabbable = false;

    bool is_captured()  const { std::lock_guard<std::mutex> g(lock); return captured;  }
    bool is_lost()      const { std::lock_guard<std::mutex> g(lock); return lost;      }
    bool is_grabbable() const { std::lock_guard<std::mutex> g(lock); return grabbable; }

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

    // Commit the three debounced flags atomically with respect to readers.
    // Writer-only entry point (loop_vision_tracking); the reader observes a
    // consistent triple via get_snapshot().
    void commit_flags(bool new_captured, bool new_lost, bool new_grabbable) {
        std::lock_guard<std::mutex> guard(lock);
        captured  = new_captured;
        lost      = new_lost;
        grabbable = new_grabbable;
    }

    struct Snapshot {
    bool   has_target;
    double bearing_rad;
    double distance_mm;
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
                 captured, lost, grabbable };
    }

    void update_frame(const cv::Mat &f, int frame_rate);
    cv::Mat get_frame() const;
    void stop() { running.store(false); }
    bool is_running() const { return running.load(); }
};


struct PathPoint {
    double x      = 0.0;     // mm, world frame
    double y      = 0.0;     // mm
    double theta  = 0.0;     // rad
    double v_max  = 150.0;   // mm/s, max cruise speed approaching this point
    double margin = 30.0;    // mm, "arrived" radius for this waypoint
};
