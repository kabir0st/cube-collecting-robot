#include <iostream>
#include <thread>
#include <fstream>
#include <atomic>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <server.hpp>
#include <exception>
#include <stdexcept>
#include <chrono>
#include <calibrate.hpp>
#include <loging.hpp>


int main() {
    LidarCalibration lidar_calibration = LidarCalibration();
    Loging loger("127.0.0.1", 9890);
    std::exception_ptr error;
    std::cout << "[info] Starting Data Listen Server . . " << std::endl;
    std::thread server_thread([&lidar_calibration, &loger, &error]() {
        try {
            auto callback = [&](int q, int a, int d) {
                lidar_calibration.collect_points(q, a, d);
                if (q) loger.collect(a, d);
            };
            if (server.start_stream(callback) != 0) {
                throw std::runtime_error("lidar stream exited with error");
            }
        } catch (const std::exception& e) {
            std::cerr << "[error] " << e.what() << std::endl;
            error = std::current_exception();
            server.stop_lidar();
        } catch (...) {
            std::cerr << "[error] unknown stream exception" << std::endl;
            error = std::current_exception();
            server.stop_lidar();
        }
    });
    std::cout << "[info] Waiting for lister to be ready." << std::endl;
    while (!server.is_ready()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    std::cout << "[info] Starting lidar now" << std::endl;
    try {
        if (server.start_lidar() != 0) {
            throw std::runtime_error("failed to start lidar");
        }
        std::cout << "[info] Calibration: collecting Lidar Points for 2s" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
    }

    float lidar_sensor_sigma = lidar_calibration.calibrate();
    std::cout << "\n Lidar sensor noise σ = " << lidar_sensor_sigma << " mm" << std::endl;

    std::cout << "[info] Streaming to visualizer. Press 'q' + Enter to quit." << std::endl;
    loger.connect_to_sink();
    loger.start();

    std::atomic<bool> quit{false};
    std::thread input_thread([&quit]() {
        char c;
        while (std::cin.get(c)) {
            if (c == 'q' || c == 'Q') { quit = true; break; }
        }
    });
    input_thread.detach();

    while (!quit) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "[info] Shutting down" << std::endl;
    server.stop_lidar();
    server_thread.join();
    loger.stop();

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
