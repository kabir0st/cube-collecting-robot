#include <ina219.h>
#include <global.hpp>
#include <types.hpp>
#include <thread>
#include <algorithm>
#include <cmath>
#include <cstdint>

#include <loggers.hpp>
#include <Lidar.hpp>
#include <utils.hpp>
#include <dashboard.hpp>
#include <calibrations.hpp>
#include <cox.hpp>
#include <motor.hpp>
#include <odometry.hpp>
#include <kalman.hpp>


void loop_check_battery(INA219& power_interface, Dashboard& dash) {
    int hits = 0;
    int absent_hits = 0;
    constexpr int ABSENT_HITS_CAP = 10;  // ~20 seconds at CHECK_PERIOD = 2s
    while (!app::shutdown) {
        float v = power_interface.supply_voltage();
        float mA = power_interface.current();
        dash.set("Battery", "Voltage", v);
        dash.set("Battery", "Current", mA);
        if (!std::isfinite(v) || !std::isfinite(mA)) {
            LOGE("Battery") << "INA219 returned non-finite reading "
                            << "(v=" << v << " mA=" << mA
                            << ") — treating as sensor fault, shutting down.";
            app::shutdown.store(true);
            break;
        }
        if (v < BATTERY_PRESENT_MIN_V) {
            hits = 0;
            if (++absent_hits >= ABSENT_HITS_CAP) {
                LOGE("Battery") << "Rail below BATTERY_PRESENT_MIN_V for "
                                << absent_hits * CHECK_PERIOD
                                << " s — INA219 likely faulted. Shutting down.";
                app::shutdown.store(true);
                break;
            }
        } else if (v <= LOW_BATTERY_LIMIT) {
            absent_hits = 0;
            hits++;
        } else {
            absent_hits = 0;
            hits = 0;
        }
        if (hits >= LOW_HITS_NEEDED)
        {
            LOGE("Battery") << "LOW (~25%): " << v << " V (" << mA
                            << " mA) — please recharge soon";
            LOGE("Battery") << "!!! Shutting down for now. !!!";
            app::shutdown.store(true);
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
}

void loop_localization(Dashboard& dash,
                       LidarData& lidar_data, KalmanFilter& kf) {
    // mapping;
    hh::Lidar lidar;
    if (!start_lidar(lidar)) {
        return;
    }
    // starting lidar data collection
    std::thread thread_loop_collect_frames(loop_collect_frames,
                std::ref(lidar), std::ref(lidar_data));
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    lidar_data.sensor_noise = get_lidar_sensor_noise(lidar_data);
    LOG("Main") << "Lidar Sensor Noise : " << lidar_data.sensor_noise;

    std::vector<LineSeg> ref_map = find_walls(lidar_data);
    if (ref_map.empty()) {
        LOGE("Main") << "find_walls produced an empty reference map; "
                        "Cox has nothing to register against. Aborting.";
        stop_lidar(lidar);
        thread_loop_collect_frames.join();
        return;
    }
    LOG("Main") << "Built reference map (" << ref_map.size() << " walls).";

    // starting cox estimation
    std::thread thread_loop_scan_cox(
        loop_scan_cox_estimation, std::ref(ref_map),
        std::ref(lidar_data),
        std::ref(dash), std::ref(kf)
    );
    // joining threads
    thread_loop_scan_cox.join();
    thread_loop_collect_frames.join();
    stop_lidar(lidar);
}


void loop_control(Dashboard& dash,
                  KalmanFilter& kf, TrackerResult& target) {
    Motor motor;
    Odometry odom;
    double total_d = 0;
    double total_a = 0;
    if (!motor.start())
    {
        LOGE("Main") << "Motor start failed";
        return;
    }
    {
        // resetting
        double l0_mm, r0_mm;
        motor.get_encoders(l0_mm, r0_mm);
        odom.reset(l0_mm, r0_mm);
    }
    while(!app::shutdown) {
        // giving time for 2 readings
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
        double l_mm, r_mm;

        motor.get_encoders(l_mm, r_mm);

        const OdomStep step = odom.update(l_mm, r_mm);
        if (step.valid) {
            kf.predict(step.d_mm, step.d_theta,
                       step.var_d, step.var_theta,
                       step.cov_d_theta);
        }
        
        double v_cmd = 0.0, w_cmd = 0.0;
        
        motor.set_velocity(v_cmd, w_cmd);

        uint64_t rejects = 0, give_ups = 0;
        motor.get_spi_stats(rejects, give_ups);

        double v_sm = 0.0, w_sm = 0.0;
        total_d += step.d_mm;
        total_a += step.d_theta;

        motor.get_current_v_w(v_sm, w_sm);

        Pos2D p;
        Pose2DCov cov;
        kf.snapshot(p, cov);
        // logger.set_kf(p, cov);

        // dashboard stats
        {
            dash.set("Odometry", "d_dist", step.d_mm/10);
            dash.set("Odometry", "d_ang", step.d_theta* 180.0 / M_PI);
            dash.set("Odometry", "Total D", total_d/10);
            dash.set("Odometry", "Total A", total_a* 180.0 / M_PI);
            // for kalman
            dash.set("KF", "X",      p.x/10);
            dash.set("KF", "Y",      p.y/10);
            dash.set("KF", "Theta", p.theta * 180.0 / M_PI);
        }

    }
    motor.stop();
}
