#pragma once
#include <atomic>
#include <functional>

class LidarDataServer {
    std::atomic<bool> listener_ready{false};
    void process_frame();

public:
    LidarDataServer() {
    // 360 / 2 ; has 2 degree separation
    // single frame = 180; taking 12 frames so
    // will process in a bit of 2160 points
    // std::vector<LidarDataFrame> frames(12);
    // processing half second of scan, 10hz scan, 
    // temp_points.reserve(4300);
    }
    
    int start_stream(std::function<void(int, int, int)> process_point);
    int start_lidar();
    int stop_lidar();
    
    bool is_ready() const {
        return listener_ready.load();
    }
    
    void set_ready(bool ready) {
        listener_ready.store(ready);
    }
};

extern LidarDataServer server;