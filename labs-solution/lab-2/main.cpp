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
#include <mapping.hpp>
#include <global.hpp>
#include <utils.hpp>

namespace app {
    std::atomic<bool> shutdown{false};
    std::atomic<bool> shutdown_listener{false};
} 
static void handle_sigint(int) { app::shutdown.store(true); }

void do_calibration() {
    std::exception_ptr calibration_error;
    LidarCalibration lidar_calibration = LidarCalibration();
    std::cout << "[calibration] Starting . . . " << std::endl;
    // waiting for server to get ready
    ListenerThread thread_listener(lidar_calibration, server);

    while (!server.is_ready())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    if (server.start_lidar() != 0) {
        throw std::runtime_error("failed to start lidar");
    }
    // run lidar for 5 seconds and collect frames
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    // stoping stream loop;
    server.stop_stream();
    // running calibration;
    float sensor_noise = lidar_calibration.calibrate();
    std::cout << "\n[RESULT] Calculated Sensor noise : " << sensor_noise << std::endl;
    lidar_calibration.create_reference_map();

    if (calibration_error)
    {
        std::rethrow_exception(calibration_error);
    }
}




int main() {
    std::signal(SIGINT, handle_sigint);
    // for finding sensor noise
    LidarCalibration lidar_calibration = LidarCalibration();
    // for sending data back to pc for visualization and logging
    Loging logger("127.0.0.1", 9890);
    std::cout << "[info] Starting Data Listen Server . . " << std::endl;
    // blocking call to do calibration
    do_calibration();
    

    // shutdown and clean up
    std::cout << "[info] Shutting down" << std::endl;
    // shutdown any rogue threads
    app::shutdown.store(true);
    logger.stop();
    return 0;
}
