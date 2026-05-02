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
#include <calibrate.hpp>
#include <loging.hpp>
#include <reference_map.hpp>
#include <cox_matcher.hpp>


int main() {
    LidarCalibration lidar_calibration = LidarCalibration();
    Loging loger("127.0.0.1", 9890);
    std::exception_ptr error;

    // Post-calibration scan accumulator. The server callback writes here
    // after `calibrated` flips; on each angle wrap we publish into latest.
    std::atomic<bool> calibrated{false};
    std::mutex frame_mutex;
    LidarDataFrame accum;
    LidarDataFrame latest;
    std::atomic<bool> latest_fresh{false};
    int last_angle = -1;

    std::cout << "[info] Starting Data Listen Server . . " << std::endl;
    std::thread server_thread([&]() {
        try {
            auto callback = [&](int q, int a, int d) {
                if (!calibrated.load()) {
                    lidar_calibration.collect_points(q, a, d);
                    if (q) loger.collect(a, d);
                    return;
                }
                if (q) loger.collect(a, d);
                if (q <= 0 || d <= 0) return;
                // Angle wrap detection: previous angle high, current low.
                if (last_angle >= 0 && a + 90 < last_angle) {
                    std::lock_guard<std::mutex> lk(frame_mutex);
                    latest.angle    = accum.angle;
                    latest.distance = accum.distance;
                    latest.num_points = static_cast<int>(accum.angle.size());
                    latest.frame   = accum.frame + 1;
                    accum.angle.clear();
                    accum.distance.clear();
                    accum.frame = latest.frame;
                    latest_fresh.store(true);
                }
                accum.angle.push_back(a);
                accum.distance.push_back(d);
                last_angle = a;
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

    std::vector<LineSeg> walls = ReferenceMapBuilder().build(lidar_calibration.samples());
    if (walls.size() != 4) {
        std::cerr << "[warn] reference map build returned " << walls.size()
                  << " walls (expected 4); pose tracking disabled" << std::endl;
    } else {
        std::cout << "[map ] reference map (4 walls):" << std::endl;
        for (std::size_t i = 0; i < walls.size(); ++i) {
            std::cout << "  wall " << i
                      << "  n=(" << walls[i].nx << ", " << walls[i].ny << ")"
                      << "  r=" << walls[i].r << " mm"
                      << "  A=(" << walls[i].ax << ", " << walls[i].ay << ")"
                      << "  B=(" << walls[i].bx << ", " << walls[i].by << ")"
                      << std::endl;
        }
    }

    // Switch the callback to scan-frame accumulator mode.
    calibrated.store(true);

    std::cout << "[info] Streaming to visualizer. Press 'q' + Enter to quit." << std::endl;
    loger.connect_to_sink();
    loger.start();

    CoxScanMatcher matcher(walls, lidar_sensor_sigma);
    Pose2D pose;
    pose.x = 0.0f; pose.y = 0.0f; pose.theta = 0.0f;
    const bool tracking_enabled = (walls.size() == 4);

    std::atomic<bool> quit{false};
    std::thread input_thread([&quit]() {
        char c;
        while (std::cin.get(c)) {
            if (c == 'q' || c == 'Q') { quit = true; break; }
        }
    });
    input_thread.detach();

    int tick = 0;
    while (!quit) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!tracking_enabled) continue;
        if (++tick < 10) continue;
        tick = 0;
        if (!latest_fresh.load()) continue;

        LidarDataFrame frame;
        {
            std::lock_guard<std::mutex> lk(frame_mutex);
            frame = latest;
            latest_fresh.store(false);
        }
        if (frame.angle.empty()) continue;

        TrackerResult r = matcher.match(frame, pose);
        if (r.converged) pose = r.pose;

        const float sx  = (r.cov[0] > 0.0f) ? std::sqrt(r.cov[0]) : 0.0f;
        const float sy  = (r.cov[4] > 0.0f) ? std::sqrt(r.cov[4]) : 0.0f;
        const float sth = (r.cov[8] > 0.0f) ? std::sqrt(r.cov[8]) : 0.0f;
        const float rxy = (sx > 0.0f && sy > 0.0f) ? r.cov[1] / (sx * sy) : 0.0f;
        const float rxt = (sx > 0.0f && sth > 0.0f) ? r.cov[2] / (sx * sth) : 0.0f;
        const float ryt = (sy > 0.0f && sth > 0.0f) ? r.cov[5] / (sy * sth) : 0.0f;

        std::cout << std::fixed << std::setprecision(3)
                  << "[pose] x=" << r.pose.x << " mm  y=" << r.pose.y
                  << " mm  th=" << r.pose.theta << " rad   iters=" << r.iterations
                  << " inliers=" << r.inliers << " conv=" << (r.converged ? 1 : 0)
                  << std::endl;
        std::cout << "[cov ] sx=" << sx << " sy=" << sy << " sth=" << sth
                  << "  rxy=" << rxy << " rxt=" << rxt << " ryt=" << ryt
                  << std::endl;
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
