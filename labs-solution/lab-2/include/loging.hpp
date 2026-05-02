#pragma once
#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <utility>

class Loging {
public:
    Loging(const char* ip = "127.0.0.1", int port = 9890);
    ~Loging();

    int connect_to_sink();
    void collect(int angle, int distance);
    void start();
    void stop();
    void sent_points();

private:
    const char* ip_;
    int port_;
    int sock_fd_;
    std::atomic<bool> connected_;
    std::atomic<bool> running_;
    std::thread worker_;
    std::mutex buf_mutex_;
    std::vector<std::pair<int,int>> buffer_;
};
