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
#include <estimate_position.hpp>
#include <global.hpp>
#include <utils.hpp>

namespace app {
    std::atomic<bool> shutdown{false};
    std::atomic<bool> shutdown_listener{false};
}
static void handle_sigint(int) { app::shutdown.store(true); }

struct CalibrationResult {
    Rectangle reference_map;
    float     sensor_noise;
};

CalibrationResult do_calibration() {
    LidarCalibration lidar_calibration;
    std::cout << "[calibration] Starting . . . " << std::endl;
    ListenerThread thread_listener(lidar_calibration, server);

    while (!server.is_ready() && !app::shutdown.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    if (server.start_lidar() != 0) {
        throw std::runtime_error("failed to start lidar");
    }
    // run lidar for 5 seconds and collect frames
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    server.stop_stream();

    CalibrationResult result;
    result.sensor_noise = lidar_calibration.calibrate();
    std::cout << "\n[RESULT] Calculated Sensor noise : "
              << result.sensor_noise << std::endl;
    result.reference_map = lidar_calibration.create_reference_map();
    return result;
}



int main() {
    std::signal(SIGINT, handle_sigint);
    Loging logger("127.0.0.1", 9890);
    std::cout << "[info] Starting Data Listen Server . . " << std::endl;

    CalibrationResult cal = do_calibration();

    if (!app::shutdown.load()) {
        do_estimate_position(cal.reference_map, cal.sensor_noise);
    }

    std::cout << "[info] Shutting down" << std::endl;
    app::shutdown.store(true);
    logger.stop();
    return 0;
}
