#pragma once
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <utility>

// Best-effort visualizer feed. Call connect_to_sink() once to dial out
// to the visualizer's TCP port; if it fails, every subsequent collect()
// call is a cheap no-op. start() spawns a worker that drains the buffer
// every 42 ms (~24 fps). stop() joins it and flushes a final time.
//
// Thread-safety:
//   * `collect` may be called from any thread; it just appends under a mutex.
//   * Only the worker thread plus stop()'s tail-flush write to the socket.
//   * `connected_` is the sole signal for "should we even try to send".
class Loging {
public:
    Loging(const char* ip = "127.0.0.1", int port = 9890);
    ~Loging();

    int  connect_to_sink();
    void collect(int angle, int distance);
    void start();
    void stop();

    bool is_connected() const { return connected_.load(); }

private:
    void flush_locked();   // called by worker and by stop()

    const char* ip_;
    int port_;
    int sock_fd_;
    std::atomic<bool> connected_;
    std::atomic<bool> running_;
    std::thread worker_;
    std::mutex buf_mutex_;
    std::vector<std::pair<int,int>> buffer_;
};
