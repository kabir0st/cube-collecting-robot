#pragma once
#include <atomic>
#include <functional>

// One small TCP server used to receive raw lidar packets from the
// vendor driver. Construct, then start_stream() on a worker thread,
// start_lidar() on the main thread (sends the start command), and
// stop_stream() to shut down. The class is a singleton in practice
// because the lidar hardware is — see `extern server` below.
class LidarDataServer {
    std::atomic<bool> listener_ready{false};
    std::atomic<bool> recv_active{false};
    std::atomic<int>  stream_server_fd{-1};
    std::atomic<int>  stream_client_fd{-1};

public:
    LidarDataServer() = default;

    // Blocks the calling thread: opens the listening socket, accepts
    // one driver connection, and reads packets until shutdown or EOF.
    // For each (quality, angle_deg, distance_mm) sample, calls process_point.
    int start_stream(std::function<void(int, int, int)> process_point);

    // Sends control codes to the driver over CONTROL_PORT.
    int start_lidar();
    int stop_lidar();

    // Stops the lidar, closes our sockets, and waits for the recv loop
    // to exit so it is safe to read whatever the listener was writing.
    int stop_stream();

    bool is_ready() const { return listener_ready.load(); }
};

// Defined in src/server.cpp.
extern LidarDataServer server;
