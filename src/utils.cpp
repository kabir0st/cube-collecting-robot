#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <map>
#include <thread>
#include <vector>

#include <global.hpp>
#include <Lidar.hpp>
#include <utils.hpp>
#include <loggers.hpp>
#include <types.hpp>

// for timestamp
#include <ctime>
#include <iomanip>

constexpr double OBSTACLE_DIST_MM      = 230.0;
constexpr double OBSTACLE_ARC_HALF_DEG = 60.0;

// Narrow cone (±GRIPPER_MASK_HALF_DEG from straight ahead) where the
// closed gripper and held box sit. Beams INSIDE this cone closer than
// GRIPPER_MASK_DIST_MM are filtered out of obstacle detection so the
// robot's own payload doesn't pin obstacle_ahead=true during CARRY_HOME
// (the closed gripper sits ~100-130 mm from the LIDAR at ±0-15°). External
// obstacles in the wider arc (±15° to ±60°) still trigger normally — a
// real wall ahead is typically wide enough to subtend more than the
// gripper cone, so a wall directly in front would still be caught by
// beams outside the mask once the robot's wide-arc lidar return reaches
// it.
constexpr double GRIPPER_MASK_HALF_DEG = 20.0;
constexpr double GRIPPER_MASK_DIST_MM  = 200.0;

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
    constexpr double INF = std::numeric_limits<double>::infinity();
    bool   frame_blocked = false;
    // Per-frame minimum distance in each half of the forward arc. Lidar 0°
    // is forward and angle increases CCW, so 0–60° lives on the body-frame
    // +Y side (left) and 300–360° on the -Y side (right). Republished at
    // each frame boundary, then reset for the next sweep.
    double left_min_mm  = INF;
    double right_min_mm = INF;
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
            app::obstacle_ahead.store(frame_blocked);
            app::obstacle_left_mm.store(left_min_mm);
            app::obstacle_right_mm.store(right_min_mm);
            frame_blocked = false;
            left_min_mm  = INF;
            right_min_mm = INF;
        }
        // only add quality reading and remove outlier dist
        if (quality > 0 && dist > 0){
            lidar_data.frames[bid].ang.push_back(angle);
            lidar_data.frames[bid].dist.push_back(dist);
            const bool in_left_arc =
                (angle > 0.0 && angle <= OBSTACLE_ARC_HALF_DEG);
            const bool in_right_arc =
                (angle >= 360.0 - OBSTACLE_ARC_HALF_DEG);
            const bool in_forward_arc = in_left_arc || in_right_arc ||
                                        angle == 0.0;
            const bool in_gripper_mask =
                (angle <= GRIPPER_MASK_HALF_DEG) ||
                (angle >= 360.0 - GRIPPER_MASK_HALF_DEG);
            // Mask: beam is the robot's own gripper / payload, not a real
            // obstacle. The gripper is stationary in the robot frame at
            // ~100-130 mm ahead, so anything inside the narrow cone closer
            // than ~200 mm is filtered. A wall behind the gripper is
            // occluded by the gripper itself anyway, so this also drops a
            // measurement that wasn't useful.
            const bool gripper_artifact =
                in_gripper_mask && dist < GRIPPER_MASK_DIST_MM;
            if (in_forward_arc && !gripper_artifact) {
                if (dist < OBSTACLE_DIST_MM) frame_blocked = true;
                if (in_left_arc)  left_min_mm  = std::min(left_min_mm,  dist);
                if (in_right_arc) right_min_mm = std::min(right_min_mm, dist);
            }
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
    // buffer_id points to the slot currently being written; the most recent
    // *complete* frame is one slot behind it. Copy only that slot under the
    // lidar_data lock — copying all FRAME_COUNT-1 frames per cox tick would
    // dominate runtime with the 50-slot buffer.
    std::lock_guard<std::mutex> guard(lidar_data.lock);
    const std::size_t n = lidar_data.frames.size();
    const std::size_t bid = static_cast<std::size_t>(lidar_data.buffer_id.load());
    const std::size_t prev = (bid + n - 1) % n;
    const Frame& f = lidar_data.frames[prev];
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

