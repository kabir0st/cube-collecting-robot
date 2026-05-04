#pragma once
#include <mutex>
#include <atomic>
#include <vector>

struct Pose2D {
    float x;        // mm
    float y;        // mm
    float theta;    // rad
};

struct LineSeg {
    float nx, ny, r;        // unit normal, n . p = r, n points away from sensor origin
    float ax, ay, bx, by;   // segment endpoints (mm)
};


constexpr int FRAME_COUNT = 20;
const char* const TCP_IP = "127.0.0.1";
constexpr int LISTENER_PORT = 9887;
constexpr int CONTORL_PORT = 9887;

const int FRAME_BUFFER_SIZE = 12;

struct LidarDataFrame
{
    int frame;
    std::vector<int> angle;
    std::vector<int> distance;
    int num_points;
    
    LidarDataFrame() : frame(0), num_points(0) {
        angle.reserve(182);
        distance.reserve(182);
    }
};

// we send a copy to SLAM process
struct Snapshot {
    std::vector<LidarDataFrame> frames;
    int lead_frame;
};

struct LidarResult {
    
    std::vector<LidarDataFrame> frames;
    int lead_frame;
    
    // we treat the lead_frame as buffer space so no need
    // to lock the whole vector space to thread safety.
    mutable std::mutex mutex_lead_frame;

    LidarResult(int num_frames = 11) : frames(num_frames), lead_frame(0) {
        for (int i = 0; i < num_frames; i++) {
            frames[i].angle.clear();
            frames[i].distance.clear();
        }
    }

    void update_lead_frame(int fc) {
        std::lock_guard<std::mutex> guard(mutex_lead_frame);
        lead_frame = fc;
    }

    Snapshot get_frames_stack() {
        std::lock_guard<std::mutex> guard(mutex_lead_frame);
        Snapshot snap;
        snap.frames = frames;      // Explicit copy
        snap.lead_frame = lead_frame;
        return snap;
        // only works in C++17 it seems
        // return {frames, lead_frame};
    }
};
