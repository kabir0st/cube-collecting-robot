#pragma once
#include <future>
#include <calibrate.hpp>
#include <server.hpp>
#include <global.hpp>

/**
 * Sends a control signal to a TCP control server.
 *
 * @param start  If non-zero, sends a START signal (0x10);
 *               otherwise sends a STOP signal (0x20).
 * @return       0 on success, -1 on failure.
 */
int send_control_signal(int start);


class ListenerThread {
    std::thread thread;

public:
    template<typename CallbackHandler>
    ListenerThread(CallbackHandler& handler, LidarDataServer& server);
    ~ListenerThread();
    // Delete copy, allow move
    ListenerThread(const ListenerThread&) = delete;
    ListenerThread& operator=(const ListenerThread&) = delete;
    // Move constructor and assignment
    ListenerThread(ListenerThread&&) = default;
    ListenerThread& operator=(ListenerThread&&) = default;
};

template<typename CallbackHandler>
ListenerThread::ListenerThread(CallbackHandler& handler, LidarDataServer& server) 
{
    thread = std::thread([this, &handler, &server]() {
        try {
            auto callback = [&handler](int q, int a, int d) {
                handler.process_point(q, a, d);
            };
            if (server.start_stream(callback) != 0) {
                throw std::runtime_error("[Error] Lidar Stream Exited.");
            }
        } catch (...) {
            std::cerr << "[Listener Thread] Stream exception" << std::endl;
            app::shutdown_listener.store(true);
            app::shutdown.store(true);
            server.stop_lidar();
        }
    });
}