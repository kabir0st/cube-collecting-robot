#include <telemetry.hpp>

#include <utils.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <sstream>
#include <thread>

namespace telemetry {

namespace {

std::mutex g_send_mtx;
int  g_sock        = -1;
bool g_enabled     = false;
int  g_subsample   = 2;
sockaddr_in g_dst{};

std::mutex g_map_mtx;
std::vector<LineSeg> g_last_map;

std::thread g_rebroadcast;
std::atomic<bool> g_run{false};

void send_raw(const std::string& s) {
    if (!g_enabled || g_sock < 0) return;
    std::lock_guard<std::mutex> lk(g_send_mtx);
    ::sendto(g_sock, s.data(), s.size(), MSG_NOSIGNAL,
             reinterpret_cast<sockaddr*>(&g_dst), sizeof(g_dst));
}

std::string build_map_packet(const std::vector<LineSeg>& walls) {
    std::ostringstream o;
    o.precision(2);
    o << std::fixed;
    o << "{\"type\":\"map\",\"walls\":[";
    for (size_t i = 0; i < walls.size(); ++i) {
        const LineSeg& w = walls[i];
        if (i) o << ',';
        o << '[' << w.ax << ',' << w.ay << ','
                 << w.bx << ',' << w.by << ']';
    }
    o << "]}\n";
    return o.str();
}

void rebroadcast_loop() {
    using namespace std::chrono;
    auto next = steady_clock::now();
    while (g_run.load()) {
        next += seconds(2);
        std::this_thread::sleep_until(next);
        if (!g_run.load()) break;
        std::vector<LineSeg> snap;
        {
            std::lock_guard<std::mutex> lk(g_map_mtx);
            snap = g_last_map;
        }
        if (!snap.empty()) send_raw(build_map_packet(snap));
    }
}

}

void init(const std::string& host, int port, bool enabled,
          int scan_subsample) {
    g_enabled   = enabled;
    g_subsample = scan_subsample > 0 ? scan_subsample : 1;
    if (!enabled) return;

    g_sock = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (g_sock < 0) { g_enabled = false; return; }

    std::memset(&g_dst, 0, sizeof(g_dst));
    g_dst.sin_family = AF_INET;
    g_dst.sin_port   = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, host.c_str(), &g_dst.sin_addr) != 1) {
        ::close(g_sock);
        g_sock = -1;
        g_enabled = false;
        return;
    }

    g_run.store(true);
    g_rebroadcast = std::thread(rebroadcast_loop);
}

void shutdown() {
    g_run.store(false);
    if (g_rebroadcast.joinable()) g_rebroadcast.join();
    if (g_sock >= 0) {
        ::close(g_sock);
        g_sock = -1;
    }
    g_enabled = false;
}

void send_map(const std::vector<LineSeg>& walls) {
    {
        std::lock_guard<std::mutex> lk(g_map_mtx);
        g_last_map = walls;
    }
    if (!g_enabled) return;
    send_raw(build_map_packet(walls));
}

void send_path(const std::vector<PathPoint>& path) {
    if (!g_enabled) return;
    std::ostringstream o;
    o.precision(4);
    o << std::fixed;
    o << "{\"type\":\"path\",\"points\":[";
    for (size_t i = 0; i < path.size(); ++i) {
        const PathPoint& p = path[i];
        if (i) o << ',';
        o << '[' << p.x << ',' << p.y << ',' << p.theta << ']';
    }
    o << "]}\n";
    send_raw(o.str());
}

void send_state(const Pos2D& pose, const Pose2DCov& cov,
                const Pos2D& goal, const PathPoint& target,
                LidarData& lidar) {
    if (!g_enabled) return;

    using namespace std::chrono;
    const int64_t t_ms = duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count();

    const double sx  = std::sqrt(std::max(0.0, cov.xx));
    const double sy  = std::sqrt(std::max(0.0, cov.yy));
    const double sth = std::sqrt(std::max(0.0, cov.tt));

    auto ang_dist = get_latest_frame_lidar_data(lidar);
    const std::vector<double>& ang  = ang_dist.first;
    const std::vector<double>& dist = ang_dist.second;

    std::ostringstream o;
    o.precision(4);
    o << std::fixed;
    o << "{\"type\":\"state\",\"t\":" << t_ms
      << ",\"pose\":[" << pose.x << ',' << pose.y << ','
                       << pose.theta << ']'
      << ",\"cov\":["  << sx << ',' << sy << ',' << sth << ']'
      << ",\"goal\":[" << goal.x << ',' << goal.y << ','
                       << goal.theta << ']'
      << ",\"target\":[" << target.x << ',' << target.y << ','
                         << target.theta << ']'
      << ",\"scan\":[";

    bool first = true;
    const size_t n = std::min(ang.size(), dist.size());
    for (size_t i = 0; i < n; i += static_cast<size_t>(g_subsample)) {
        const double a = ang[i];
        const double r = dist[i];
        if (!std::isfinite(a) || !std::isfinite(r) || r <= 0.0) continue;
        if (!first) o << ',';
        first = false;
        o << '[' << a << ',' << r << ']';
    }
    o << "]}\n";
    send_raw(o.str());
}

}
