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
        static constexpr double LEFT_TICKS_PER_MM  = 122.82;
        static constexpr double RIGHT_TICKS_PER_MM = 123.26;
        static constexpr double WHEELBASE_MM       = 175.0;

        Motor();
        ~Motor();
        bool start();
        // Requests a smooth ramp to zero before shutting the IO thread down.
        void stop();

        //   v_left  = v - omega * WHEELBASE_MM/2
        //   v_right = v + omega * WHEELBASE_MM/2
        void set_velocity(double v_mm_s, double omega_rad_s);

        void get_encoders(double& left_mm, double& right_mm) const;

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

        double _v_target = 0.0, _w_target = 0.0;
        double _v_cur    = 0.0, _w_cur    = 0.0;

        std::atomic<bool> _motor_running{false};
        std::thread _io;

};
