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
#include <utils.cpp>


using namespace std;

LidarDataServer server;



void LidarDataServer::process_frame() {
    
}



int LidarDataServer::start_stream() {
    int server = -1;
    int client = -1;
    ofstream data_file;
    // 360 / 2 ; has 2 degree separation
    // single frame = 180; taking 12 frames so
    // will process in a bit of 2160 points
    std::vector<LidarDataFrame> frames(12);

    try {
        cout << "[info] Starting lidar driver" << endl;
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


        // Open file for writing lidar data
        std::stringstream filename;
        filename << "lidar_data.txt";
        
        data_file.open(filename.str(), std::ios::out);
        if (!data_file.is_open()) {
            throw std::runtime_error("failed to open lidar data file");
        }
        std::cout << "[info] Writing lidar data to: " << filename.str() << std::endl;
        
        // Write header to file
        data_file << "# Format: quality angle distance" << std::endl;
        data_file << "#" << std::endl;
        data_file.flush();

        char header[5];
        char data[4096];
        long frame_count;
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
                if (quality > 0) {
                    int angle = (((unsigned char)data[1] >> 1) +
                    ((unsigned char)data[2] << 8)) >> 7;
                    int distance = ((unsigned char)data[3] +
                    ((unsigned char)data[4] << 8)) >> 2;
                    // process frames here
                    // Write data to file instead of printing
                    data_file << quality << " " << angle << " " << distance << std::endl;
                    frame_count++;
                    // Flush periodically to ensure data is written
                    if (frame_count % 200 == 0) {
                        data_file.flush();
                        std::cout << "[info] Written " << frame_count << " frames" << std::endl;
                        frame_count = 0;
                    }
                }
            }
        }
        listener_ready.store(false);
        close(client);
        close(server);
        return 0;
    } catch (const std::exception& e) {
        if (client >= 0) close(client);
        if (server >= 0) close(server);
        std::cerr << "[error] " << e.what() << std::endl;
        return -1;
    }
}

int LidarDataServer::start_lidar() {
    try {

        while(!listener_ready.load()) {
            cout << "[info] Listener not ready, waiting 2 seconds. " << endl;
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
