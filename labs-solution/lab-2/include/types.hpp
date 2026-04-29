#pragma once
#include <mutex>
#include <atomic>
#include <vector>

constexpr int FRAME_COUNT = 20;
const char* const TCP_IP = "127.0.0.1";
constexpr int LISTENER_PORT = 9887;
constexpr int CONTORL_PORT = 9887;

std::atomic<bool> listener_ready{false};


struct LidarDataFrame
{
    int frame;
    std::vector<float> angle;
    std::vector<float> distance;
    int num_points;
    
    LidarDataFrame() : frame(0), num_points(0) {
        angle.reserve(182);
        distance.reserve(182);
    }
};

struct LidarResult {
    std::vector<LidarDataFrame> frames;
    int lead_frame;
};
