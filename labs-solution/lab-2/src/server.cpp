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

// for networking
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
LidarDataServer server;



void LidarDataServer::process_frame() {
    
}



int LidarDataServer::start_stream() {
    int server = -1;
    int client = -1;
    // cannot do implicit copy cause of C++11
    LidarResult lidar_frame_stack(FRAME_BUFFER_SIZE);
    int last_angle = -1;
    bool is_new_frame = false;

    try
    {
        std::cout << "[info] Starting lidar driver" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        server = socket(AF_INET, SOCK_STREAM, 0);
        if (server < 0) {
            throw std::runtime_error("failed to create lidar stream socket");
        }

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

        std::cout << "[info] Waiting for lidar client..." << std::endl;
        listener_ready.store(true);
        client = accept(server, nullptr, nullptr);
        if (client < 0) {
            throw std::runtime_error("failed to accept lidar client");
        }
        std::cout << "[info] Lidar client connected" << std::endl;

        char header[5];
        char data[4096];
        while (recv(client, header, 5, 0) == 5)
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
                    
                if (angle == 0) {
                    if (last_angle != 0 ) {
                        is_new_frame = true;
                    }
                }
                last_angle = angle;
                if (quality > 0) {
                    int distance = ((unsigned char)data[3] +
                    ((unsigned char)data[4] << 8)) >> 2;
                    if (is_new_frame) {
                        // thread safe
                        lidar_frame_stack.update_lead_frame((lidar_frame_stack.lead_frame + 1) % 11);
                        // Clear the new lead frame before using it
                        lidar_frame_stack.frames[lidar_frame_stack.lead_frame].angle.clear();
                        lidar_frame_stack.frames[lidar_frame_stack.lead_frame].distance.clear();
                        is_new_frame = false;
                    }
                    lidar_frame_stack.frames[lidar_frame_stack.lead_frame].angle.push_back(angle);
                    lidar_frame_stack.frames[lidar_frame_stack.lead_frame].distance.push_back(distance);
                }
            }
        }
        listener_ready.store(false);
        close(client);
        close(server);
        return 0;
    }
    catch (const std::exception &e)
    {
        if (client >= 0) close(client);
        if (server >= 0) close(server);
        std::cerr << "[error] " << e.what() << std::endl;
        return -1;
    }
}

int LidarDataServer::start_lidar() {
    try {

        while(!listener_ready.load()) {
            std::cout << "[info] Listener not ready, waiting 2 seconds. " << std::endl;
            std::this_thread::sleep_for(
                std::chrono::seconds(2));
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (send_control_signal(1) != 0) {
            throw std::runtime_error("error while sending start signal");
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "[info] Lidar start signal sent" << std::endl;
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
        std::cout << "[info] Lidar stop signal sent" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
        return -1;
    }
}
