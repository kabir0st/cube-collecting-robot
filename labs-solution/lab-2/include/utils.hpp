#pragma once
#include <thread>
#include <iostream>
#include <stdexcept>

#include <server.hpp>
#include <global.hpp>
#include <log.hpp>

// Send a one-byte control signal to the lidar driver on CONTROL_PORT.
//   start != 0  -> 0x10 (start spinning + scanning)
//   start == 0  -> 0x20 (stop)
// Returns 0 on success, -1 on failure.
int send_control_signal(int start);


// Owns one std::thread that runs LidarDataServer::start_stream and
// forwards each (quality, angle, distance) sample to handler.process_point.
// Joined in the destructor so the listener never outlives its handler.
class ListenerThread {
    std::thread worker_;

public:
    template<typename Handler>
    ListenerThread(Handler& handler, LidarDataServer& srv) {
        // No `this` capture — keeps move semantics safe and the lambda
        // independent of the wrapper object's lifetime.
        worker_ = std::thread([&handler, &srv]() {
            try {
                srv.start_stream(
                    [&handler](int q, int a, int d) {
                        handler.process_point(q, a, d);
                    });
            } catch (...) {
                LOGE("listener") << "unhandled exception";
                app::shutdown.store(true);
            }
        });
    }

    // Idempotent. After this returns, no more handler.process_point calls
    // are in flight from this listener's thread.
    void join() {
        if (worker_.joinable()) worker_.join();
    }

    ~ListenerThread() { join(); }

    ListenerThread(const ListenerThread&)            = delete;
    ListenerThread& operator=(const ListenerThread&) = delete;
    ListenerThread(ListenerThread&&)                 = default;
    ListenerThread& operator=(ListenerThread&&)      = default;
};
