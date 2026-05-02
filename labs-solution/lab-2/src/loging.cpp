#include <loging.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstdint>
#include <cstring>
#include <chrono>
#include <iostream>

Loging::Loging(const char* ip, int port)
    : ip_(ip), port_(port), sock_fd_(-1),
      connected_(false), running_(false) {
    buffer_.reserve(2048);
}

Loging::~Loging() {
    stop();
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
}

int Loging::connect_to_sink() {
    sock_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd_ < 0) {
        std::cerr << "[warn] Loging: socket() failed" << std::endl;
        return 1;
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    if (::inet_pton(AF_INET, ip_, &addr.sin_addr) <= 0) {
        std::cerr << "[warn] Loging: bad ip " << ip_ << std::endl;
        ::close(sock_fd_);
        sock_fd_ = -1;
        return 1;
    }
    if (::connect(sock_fd_, (sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "[warn] Loging: connect to " << ip_ << ":" << port_
                  << " failed (visualizer not running?)" << std::endl;
        ::close(sock_fd_);
        sock_fd_ = -1;
        return 1;
    }
    connected_ = true;
    std::cout << "[info] Loging connected to " << ip_ << ":" << port_ << std::endl;
    return 0;
}

void Loging::collect(int angle, int distance) {
    if (!connected_) return;
    std::lock_guard<std::mutex> g(buf_mutex_);
    buffer_.emplace_back(angle, distance);
}

void Loging::start() {
    if (running_) return;
    running_ = true;
    worker_ = std::thread([this]() {
        while (running_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(42));
            sent_points();
        }
    });
}

void Loging::stop() {
    if (!running_) return;
    running_ = false;
    if (worker_.joinable()) worker_.join();
    sent_points();
}

void Loging::sent_points() {
    if (!connected_) return;

    std::vector<std::pair<int,int>> batch;
    {
        std::lock_guard<std::mutex> g(buf_mutex_);
        if (buffer_.empty()) return;
        batch.swap(buffer_);
    }

    const uint32_t n = static_cast<uint32_t>(batch.size());
    std::vector<int32_t> payload;
    payload.reserve(1 + 2 * n);
    payload.push_back(static_cast<int32_t>(n));
    for (const auto& p : batch) {
        payload.push_back(static_cast<int32_t>(p.first) * 100);
        payload.push_back(static_cast<int32_t>(p.second));
    }

    const char* data = reinterpret_cast<const char*>(payload.data());
    size_t total = payload.size() * sizeof(int32_t);
    size_t sent = 0;
    while (sent < total) {
        ssize_t r = ::send(sock_fd_, data + sent, total - sent, MSG_NOSIGNAL);
        if (r <= 0) {
            std::cerr << "[warn] Loging: send failed, disabling stream" << std::endl;
            connected_ = false;
            ::close(sock_fd_);
            sock_fd_ = -1;
            return;
        }
        sent += static_cast<size_t>(r);
    }
}
