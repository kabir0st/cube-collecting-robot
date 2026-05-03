#include <iostream>
#include <iomanip>
#include <thread>
#include <fstream>
#include <atomic>
#include <mutex>
#include <cmath>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <server.hpp>
#include <exception>
#include <stdexcept>
#include <chrono>
#include <csignal>
#include <calibrate.hpp>
#include <loging.hpp>
#include <reference_map.hpp>
#include <global.hpp>

namespace app {
    std::atomic<bool> shutdown{false};
    std::atomic<bool> shutdown_listener{false};
} 
static void handle_sigint(int) { app::shutdown.store(true); }



void stream_callback_loop(int q, int a, int d){

    // to check if sensor calibration is needed or not
    std::atomic<bool> calibrated{false};
    std::mutex frame_mutex;
    LidarDataFrame accum;
    LidarDataFrame latest;
    std::atomic<bool> latest_fresh{false};
    int last_angle = -1;

    if (a == 0) {
        std::cout << "\rAngle : " << d << std::flush;
    }
}


void do_calibration() {
    std::exception_ptr calibration_error;

    LidarCalibration lidar_calibration = LidarCalibration();
    std::cout << "[calibration] Starting . . . " << std::endl;
    std::thread server_thread([&]() {
        try {
            if (server.start_stream(stream_callback_loop) != 0) {
                throw std::runtime_error("lidar stream exited with error");
            }
        } catch (const std::exception& e) {
            std::cerr << "[error] " << e.what() << std::endl;
            calibration_error = std::current_exception();
            app::shutdown.store(true);
            server.stop_lidar();
        } catch (...) {
            std::cerr << "[error] unknown stream exception" << std::endl;
            calibration_error = std::current_exception();
            app::shutdown.store(true);
            server.stop_lidar();
        }
    });
}

int main() {
    std::signal(SIGINT, handle_sigint);
    // for finding sensor noise
    LidarCalibration lidar_calibration = LidarCalibration();
    // for sending data back to pc for visualization and logging
    Loging logger("127.0.0.1", 9890);
    std::exception_ptr error;
    std::cout << "[info] Starting Data Listen Server . . " << std::endl;
    // blocking call to do calibration
    do_calibration();

    // // main thread
    // std::thread server_thread([&]() {
    //     try {
    //         if (server.start_stream(stream_callback_loop) != 0) {
    //             throw std::runtime_error("lidar stream exited with error");
    //         }
    //     } catch (const std::exception& e) {
    //         std::cerr << "[error] " << e.what() << std::endl;
    //         error = std::current_exception();
    //         app::shutdown.store(true);
    //         server.stop_lidar();
    //     } catch (...) {
    //         std::cerr << "[error] unknown stream exception" << std::endl;
    //         error = std::current_exception();
    //         app::shutdown.store(true);
    //         server.stop_lidar();
    //     }
    // });

    
    // starting lidar after starting listener
    std::cout << "[info] Waiting for lister to be ready." << std::endl;
    while (!server.is_ready()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    std::cout << "[info] Starting lidar now" << std::endl;
    try {
        if (server.start_lidar() != 0) {
            throw std::runtime_error("failed to start lidar");
        }
        std::cout << "[info] Started Lidar Driver." << std::endl;
        // main event loop
        std::cout << "[info] Going into main event loop." << std::endl;
        while (!app::shutdown.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
    }

    std::cout << "[info] Shutting down" << std::endl;
    app::shutdown.store(true);
    server.stop_lidar();
    server.stop_stream();
    server_thread.join();
    logger.stop();

    if (error) {
        try {
            std::rethrow_exception(error);
        } catch (const std::exception& e) {
            std::cerr << "[error] " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[error] unknown exception" << std::endl;
        }
    }
    return 0;
}
