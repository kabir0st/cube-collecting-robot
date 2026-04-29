#pragma once
#include <atomic>

class LidarDataServer {
    std::atomic<bool> listener_ready{false};
    void process_frame();

public:
    int start_stream();
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