#include <iostream>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <functional>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <types.hpp>
#include <server.hpp>
#include <utils.hpp>
#include <global.hpp>
#include <log.hpp>


LidarDataServer server;


// Accept one driver connection on LIDAR_STREAM_PORT and read RPLIDAR
// "express scan" packets until shutdown or EOF.
//
// Packet layout (5-byte header + payload):
//   header[0]    = 0xA5  (start byte; we only handle this kind)
//   header[1..4] = misc + 24-bit big-endian payload size
//   payload[0]   = (quality << 2) | flags
//   payload[1..2]= angle in q6 fixed-point (1/64 deg), little-endian
//   payload[3..4]= distance in q2 fixed-point (1/4 mm), little-endian
//
// We coarsen angle to integer degrees and distance to integer mm before
// pushing to process_point — the consumers downstream don't need more.
int LidarDataServer::start_stream(std::function<void(int, int, int)> process_point) {
    int srv = -1, cli = -1;
    try {
        // Brief delay so the driver has time to start its control listener
        // before we try to accept its connection.
        std::this_thread::sleep_for(std::chrono::seconds(1));

        srv = ::socket(AF_INET, SOCK_STREAM, 0);
        if (srv < 0) throw std::runtime_error("socket() failed");
        stream_server_fd.store(srv);

        // SO_REUSEADDR so a quick restart doesn't trip on TIME_WAIT.
        int yes = 1;
        ::setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

        sockaddr_in addr{};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = ::inet_addr(TCP_IP);
        addr.sin_port        = htons(LIDAR_STREAM_PORT);
        if (::bind(srv, (sockaddr*)&addr, sizeof(addr)) < 0)
            throw std::runtime_error("bind() failed");
        if (::listen(srv, 1) < 0)
            throw std::runtime_error("listen() failed");

        LOG("stream") << "waiting for lidar driver...";
        listener_ready.store(true);

        cli = ::accept(srv, nullptr, nullptr);
        if (cli < 0) {
            // accept() can fail because stop_stream() closed the socket
            // out from under us — that's a clean shutdown, not an error.
            if (!app::shutdown.load())
                throw std::runtime_error("accept() failed");
        } else {
            stream_client_fd.store(cli);
            LOG("stream") << "lidar client connected";

            recv_active.store(true);
            char header[5], data[4096];
            std::size_t pkt_count = 0;
            while (!app::shutdown.load() && ::recv(cli, header, 5, 0) == 5) {
                if ((unsigned char)header[0] != 0xA5) continue;

                const int size = ((unsigned char)header[2] << 16)
                               | ((unsigned char)header[3] <<  8)
                               |  (unsigned char)header[4];
                if (size <= 0 || size > (int)sizeof(data))
                    throw std::runtime_error("invalid lidar packet size");
                if (::recv(cli, data, size, 0) != size)
                    throw std::runtime_error("short read on lidar payload");

                const int quality   = (unsigned char)data[0] >> 2;
                const int angle_q6  = ((unsigned char)data[1] >> 1)
                                    | ((unsigned char)data[2] << 7);
                const int angle_deg = (angle_q6 >> 6) % 360;
                const int dist_mm   = (((unsigned char)data[3])
                                     | ((unsigned char)data[4] << 8)) >> 2;
                process_point(quality, angle_deg, dist_mm);

                if ((++pkt_count % 4096u) == 0)
                    LOG("stream") << "heartbeat: " << pkt_count << " pkts";
            }
        }
    } catch (const std::exception& e) {
        LOGE("stream") << "error: " << e.what();
    }

    // Common cleanup — runs on success, EOF, exception, and shutdown.
    listener_ready.store(false);
    int s = stream_server_fd.exchange(-1);
    int c = stream_client_fd.exchange(-1);
    if (c >= 0) ::close(c);
    if (s >= 0) ::close(s);

    // Final flag store: only meaningful for diagnostics now that
    // synchronization is via std::thread::join() in ListenerThread.
    recv_active.store(false);
    return 0;
}


int LidarDataServer::stop_stream() {
    // Tell the driver to spin down before we yank its socket — avoids
    // a brief flood of orphan packets.
    stop_lidar();

    // Close from under the recv loop. shutdown() unblocks accept()/recv()
    // with EOF so start_stream() returns promptly. The caller is
    // responsible for joining the listener thread after this — that is
    // the only correct sync point for "the handler is no longer in use".
    int s = stream_server_fd.exchange(-1);
    int c = stream_client_fd.exchange(-1);
    if (c >= 0) { ::shutdown(c, SHUT_RDWR); ::close(c); }
    if (s >= 0) { ::shutdown(s, SHUT_RDWR); ::close(s); }
    return 0;
}


int LidarDataServer::start_lidar() {
    // Wait until our listener socket is ready, otherwise the driver
    // would race past our accept().
    while (!listener_ready.load() && !app::shutdown.load()) {
        LOG("stream") << "listener not ready, waiting...";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (app::shutdown.load()) return -1;

    if (send_control_signal(1) != 0) {
        LOGE("stream") << "failed to send start signal";
        return -1;
    }
    LOG("stream") << "lidar start signal sent";
    return 0;
}


int LidarDataServer::stop_lidar() {
    if (send_control_signal(0) != 0) {
        LOGE("stream") << "failed to send stop signal";
        return -1;
    }
    LOG("stream") << "lidar stop signal sent";
    return 0;
}
