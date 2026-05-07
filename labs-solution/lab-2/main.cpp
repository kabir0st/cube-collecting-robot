#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

#include <execinfo.h>
#include <unistd.h>

#include <calibrate.hpp>
#include <estimate_position.hpp>
#include <global.hpp>
#include <log.hpp>
#include <loging.hpp>
#include <mapping.hpp>
#include <server.hpp>
#include <utils.hpp>


// =============================================================================
// Top-level driver
// =============================================================================
//
// Two-phase program:
//   1. Calibration  — keep the lidar still for ~5 s, learn sensor sigma and
//      build the reference map (a list of wall segments) via RANSAC.
//   2. Estimation   — match each new revolution against the reference map at
//      1 Hz, print pose with covariance.
//
// Both phases share the same lidar driver and TCP plumbing (see server.cpp).


namespace app {
    std::atomic<bool> shutdown{false};
}


static void handle_sigint(int) { app::shutdown.store(true); }


// Print a backtrace and abort. Installed as terminate handler so that
// "pure virtual called", uncaught exceptions, and other terminate-paths
// land here with at least a thread id and a trace instead of bare
// "terminate called without an active exception".
static void on_terminate() {
    LOGE("fatal") << "std::terminate invoked";
    void* frames[64];
    int n = ::backtrace(frames, 64);
    ::backtrace_symbols_fd(frames, n, STDERR_FILENO);
    std::abort();
}


struct CalibrationResult {
    std::vector<LineSeg> reference_map;
    float                sensor_noise;
};


static CalibrationResult do_calibration(Loging* logger) {
    LidarCalibration cal(logger);
    LOG("cal") << "starting . . .";

    ListenerThread listener(cal, server);
    while (!server.is_ready() && !app::shutdown.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (app::shutdown.load()) {
        server.stop_stream();
        listener.join();
        return {};
    }
    if (server.start_lidar() != 0) {
        server.stop_stream();
        listener.join();
        throw std::runtime_error("failed to start lidar for calibration");
    }

    // 5 s of stationary capture.
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Order matters: stop_stream() unblocks the recv loop; listener.join()
    // is the synchronization point that guarantees no further
    // cal.process_point() call is in flight before we read cal below.
    server.stop_stream();
    listener.join();

    CalibrationResult res;
    res.sensor_noise = cal.calibrate();
    LOG("cal") << "sigma = " << res.sensor_noise << " mm";

    res.reference_map = cal.create_reference_map();
    return res;
}


int main() {
    std::set_terminate(&on_terminate);
    std::signal(SIGINT, handle_sigint);

    Loging logger("127.0.0.1", 9890);
    logger.connect_to_sink();   // best-effort; visualizer optional
    logger.start();

    LOG("main") << "starting";

    CalibrationResult cal = do_calibration(&logger);
    if (app::shutdown.load() || cal.reference_map.empty()) {
        LOG("main") << "aborting: calibration not completed";
        logger.stop();
        return 0;
    }

    do_estimate_position(cal.reference_map, cal.sensor_noise, &logger);

    LOG("main") << "shutting down";
    app::shutdown.store(true);
    logger.stop();
    return 0;
}
