#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>

#include <spi_com.h>

// right-wheel wiring inversion are all hidden inside this class.
class Motor {
    public:
        // Drive-train constants. Per-wheel calibration: the encoders +
        // wheels are not identical (typical ~2% asymmetry from wheel-radius
        // variation), and feeding both raw deltas through one shared scale
        // injects a fake heading change every step that compounds into
        // unbounded drift. Calibrate each wheel independently against the
        // same ground-truth distance and update here.
        //
        // Calibration method: push the robot in a straight line over a
        // tape-measured distance (e.g. 1 m), read raw ticks per wheel,
        // ticks_per_mm = raw_ticks / actual_mm.
        static constexpr double LEFT_TICKS_PER_MM  = 121.04;
        static constexpr double RIGHT_TICKS_PER_MM = 118.42;
        static constexpr double WHEELBASE_MM       = 175.0;

        Motor();
        ~Motor();
        bool start();
        // Requests a smooth ramp to zero before shutting the IO thread down.
        void stop();

        // Commanded chassis target: v forward in mm/s, omega CCW in rad/s.
        // Stored as the *target* — the 50 Hz IO loop ramps the actually-sent
        // wheel command toward it under fixed acceleration limits
        // (ACCEL_V_MM_S2, ACCEL_W_RAD_S2 in motor.cpp). Diff-drive inverse
        // kinematics happen inside the loop:
        //   v_left  = v - omega * WHEELBASE_MM/2
        //   v_right = v + omega * WHEELBASE_MM/2
        void set_velocity(double v_mm_s, double omega_rad_s);

        // Cumulative wheel travel in mm since boot, forward = + for both
        // wheels (no per-call offset subtraction is applied today).
        void get_encoders(double& left_mm, double& right_mm) const;

        // Smoothed chassis command currently being sent (post-slew, post-
        // filter). For dashboard / telemetry — this is what the wheels are
        // actually seeing, not the upstream target.
        void get_current_v_w(double& v_mm_s, double& omega_rad_s) const;

        void get_spi_stats(uint64_t& rejects) const;

    private:
        void _loop_motor_read_write();
        mutable std::mutex _mutex_motor;
        int64_t _enc_left_acc  = 0;
        int64_t _enc_right_acc = 0;

        int32_t _last_acc_left = 0, _last_acc_right = 0;
        std::chrono::steady_clock::time_point _last_acc_ts;
        uint64_t _spi_reject_count = 0;

        // Setpoint smoother. _v_target/_w_target are the upstream request;
        // _v_cur/_w_cur are what the IO loop ramps and actually sends.
        double _v_target = 0.0, _w_target = 0.0;
        double _v_cur    = 0.0, _w_cur    = 0.0;

        std::atomic<bool> _motor_running{false};
        std::thread _io;

};
