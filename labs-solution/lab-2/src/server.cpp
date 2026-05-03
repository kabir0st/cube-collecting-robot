#include <iostream>
#include <mutex>
#include <atomic>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <types.hpp>
#include <server.hpp>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <utils.hpp>
#include <global.hpp>

// for networking
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <functional>

LidarDataServer server;


int LidarDataServer::start_stream(std::function<void(int, int, int)> process_point) {
    int server = -1;
    int client = -1;
    // cannot do implicit copy cause of C++11
    LidarResult lidar_frame_stack(FRAME_BUFFER_SIZE);

    try
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        server = socket(AF_INET, SOCK_STREAM, 0);
        if (server < 0) {
            throw std::runtime_error("failed to create lidar stream socket");
        }
        stream_server_fd.store(server);

        sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        addr.sin_port = htons(9888);

        if (bind(server, (sockaddr*)&addr, sizeof(addr)) < 0) {
            throw std::runtime_error("failed to bind lidar stream socket");
        }
        if (listen(server, 1) < 0) {
            throw std::runtime_error("failed to listen on lidar stream socket");
        }

        std::cout << "[Stream Thread] Waiting for lidar driver..." << std::endl;
        listener_ready.store(true);
        client = accept(server, nullptr, nullptr);
        if (client < 0) {
            if (app::shutdown.load()) {
                listener_ready.store(false);
                int s = stream_server_fd.exchange(-1);
                if (s >= 0) close(s);
                return 0;
            }
            throw std::runtime_error("failed to accept lidar client");
        }
        stream_client_fd.store(client);
        std::cout << "[Steam Thread] Lidar client connected" << std::endl;

        char header[5];
        char data[4096];
        while (!app::shutdown.load() && recv(client, header, 5, 0) == 5)
        {
            if ((unsigned char)header[0] == 165) {
                int size = ((unsigned char)header[2] << 16) |
                        ((unsigned char)header[3] << 8) |
                        (unsigned char)header[4];
                if (size <= 0 || size > 4096) {
                    throw std::runtime_error("invalid lidar packet size");
                }

                if (recv(client, data, size, 0) != size) {
                    throw std::runtime_error("failed to read lidar packet");
                }

                int quality = (unsigned char)data[0] >> 2;
                int angle = (((unsigned char)data[1] >> 1) +
                    ((unsigned char)data[2] << 8)) >> 7;
                int distance = ((unsigned char)data[3] +
                ((unsigned char)data[4] << 8)) >> 2;
                process_point(quality, angle, distance);
            }
        }
        listener_ready.store(false);
        int c = stream_client_fd.exchange(-1);
        int s = stream_server_fd.exchange(-1);
        if (c >= 0) close(c);
        if (s >= 0) close(s);
        return 0;
    } catch (const std::exception &e) {
        listener_ready.store(false);
        int c = stream_client_fd.exchange(-1);
        int s = stream_server_fd.exchange(-1);
        if (c >= 0) close(c);
        if (s >= 0) close(s);
        std::cerr << "[error] " << e.what() << std::endl;
        return -1;
    }
}

int LidarDataServer::stop_stream() {
    // stoping lidar before stopping stream
    this->stop_lidar();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    int s = stream_server_fd.exchange(-1);
    int c = stream_client_fd.exchange(-1);
    if (c >= 0) {
        ::shutdown(c, SHUT_RDWR);
        ::close(c);
    }
    if (s >= 0) {
        ::shutdown(s, SHUT_RDWR);
        ::close(s);
    }
    return 0;
}

int LidarDataServer::start_lidar() {
    try {

        while(!listener_ready.load()) {
            std::cout << "[Steam Thread] Listener not ready, waiting 2 seconds. " << std::endl;
            std::this_thread::sleep_for(
                std::chrono::seconds(2));
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (send_control_signal(1) != 0) {
            throw std::runtime_error("error while sending start signal");
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "[Steam Thread] Lidar start signal sent" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        return -1;
    }
}


int LidarDataServer::stop_lidar() {
    try {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (send_control_signal(0) != 0) {
            throw std::runtime_error("error while sending stop signal");
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "[Steam Thread] Lidar stop signal sent" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        return -1;
    }
}
