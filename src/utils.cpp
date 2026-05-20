#include <iostream>
#include <thread>
#include <chrono>
#include <map>
#include <vector>

#include <global.hpp>
#include <Lidar.hpp>
#include <utils.hpp>
#include <loggers.hpp>
#include <types.hpp>

// for timestamp
#include <ctime>
#include <iomanip>

bool start_lidar(hh::Lidar& lidar) {
    if (lidar.connect() != LIDAR_SUCCESS) {
        LOGE("LIDAR") << "Failed to connect to lidar on " << LIDAR_SERIAL_DEVICE;
        return false;
    }
    // Reset state in case a previous run was killed mid-scan and left the motor spinning.
    lidar.stop();
    lidar.setMotorPwm(LIDAR_IDLE_MOTOR_PWM);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    lidar.setMotorPwm(LIDAR_DEFAULT_MOTOR_PWM);
    if (lidar.scan() != LIDAR_SUCCESS) {
        LOGE("LIDAR") << "Failed to start scan";
        lidar.setMotorPwm(LIDAR_IDLE_MOTOR_PWM);
        return false;
    }
    // waiting for lidar to spin up properly
    std::this_thread::sleep_for(std::chrono::seconds(2));
    app::lidar_ready.store(true);
    app::lidar_stop.store(false);
    return true;
}

void stop_lidar(hh::Lidar& lidar) {
    app::lidar_ready.store(false);
    lidar.stop();
    lidar.setMotorPwm(LIDAR_IDLE_MOTOR_PWM);
    LOG("LIDAR") << "Lidar stopped.";
    app::lidar_stop.store(true);
    // waiting for lidar's motor to stop completely
    std::this_thread::sleep_for(std::chrono::seconds(2));
    return;
}

void loop_collect_frames(hh::Lidar& lidar, LidarData& lidar_data) {
    lidar_normal_scan_data_packet_t pkt;
    while (!app::lidar_ready && !app::shutdown) {
        //waiting for lidar to get ready
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (app::shutdown) return;
    LOG("LIDAR") << "Starting frame collection";
    while (!app::shutdown && !app::lidar_stop)
    {
        if (lidar.readNormalScanDataPacket(&pkt) != LIDAR_SUCCESS) continue;
        double angle = ((pkt.byte2 << 7) | (pkt.byte1 >> 1)) / 64.0;
        double dist  = ((pkt.byte4 << 8) |  pkt.byte3)       / 4.0;
        int quality = pkt.byte0 >> 2;
        // Detect frame boundary by a clear backwards step in angle
        int bid = lidar_data.buffer_id.load();
        auto& active_ang = lidar_data.frames[bid].ang;
        // to handle jitter angle readings
        if (!active_ang.empty() && angle + 3.0 < active_ang.back()) {
            lidar_data.update_buffer_id();
            bid = lidar_data.buffer_id.load();
        }
        // only add quality reading and remove outlier dist
        if (quality > 0 && dist > 0){
            lidar_data.frames[bid].ang.push_back(angle);
            lidar_data.frames[bid].dist.push_back(dist);
        }
    }
}


std::map<double, std::vector<double>> get_angle_mapped_lidar_data(LidarData& lidar_data) {
    std::array<Frame, FRAME_COUNT - 1> latest_frames = lidar_data.get_latest_snapshot();
    std::map<double, std::vector<double>> grouped;
    std::size_t total = 0;

    for (const Frame& f : latest_frames) {
        const std::size_t n = std::min(f.ang.size(), f.dist.size());
        total += n;
        for (std::size_t i = 0; i < n; ++i) {
            grouped[f.ang[i]].push_back(f.dist[i]);
        }
    }
    return grouped;
}


std::pair<std::vector<double>, std::vector<double>> get_latest_frame_lidar_data(LidarData& lidar_data) {
    std::array<Frame, FRAME_COUNT - 1> latest_frames = lidar_data.get_latest_snapshot();
    // get_latest_snapshot returns frames oldest -> newest; the last entry
    // is the freshest complete frame.
    const Frame& f = latest_frames.back();
    return { std::vector<double>(f.ang.begin(), f.ang.end()),
             std::vector<double>(f.dist.begin(), f.dist.end()) };
}

INA219 configure_voltage_interface() {
    INA219 Ina(SHUNT_OHMS, MAX_EXPECTED_AMPS);
    Ina.configure(RANGE_16V, GAIN_8_320MV, ADC_12BIT, ADC_12BIT);
    return Ina;
}


std::string get_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()
    ) % 1000;
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::setfill('0')
        << std::setw(2) << tm.tm_hour << ":"
        << std::setw(2) << tm.tm_min << ":"
        << std::setw(2) << tm.tm_sec << "."
        << std::setw(3) << ms.count();
    return oss.str();
}

