#include <motor.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <thread>
#include <global.hpp>
#include <loggers.hpp>
#include <spi_com.h>
#include <wiringPiSPI.h>

namespace {
    // Physical ceiling on encoder ticks/sec at the motor shaft.
    // Nominal motor 4670 rpm × 4096 quad ticks/rev ≈ 319,000 ticks/s.
    // 400,000 sits comfortably above that.
    // (Gearing: motor → Z24 → Z40 on wheel shaft, 5/3 reduction.)
    constexpr double MAX_TICKS_PER_SEC = 400000.0;
    // Hard ceiling on commanded per-wheel pps sent to the Arduino.
    // Calibrated to cap wheel-frame speed at ~400 mm/s under the
    // current TICKS_PER_MM (71.48), leaving ~54 % headroom over the
    // controller's worst-case wheel-frame speed (~260 mm/s).
    //   pps cap = 400 mm/s × TICKS_PER_MM = 28,592 ticks/s = 286 pps100.
    constexpr double MAX_CMD_PPS100    = 286.0;
    // jitter margin
    constexpr double SAFETY_FACTOR     = 2.0;
    // Cap on the time window used for max_delta. Prevents the per-tick
    // physical-travel threshold from growing without bound across a
    // burst of bad SPI reads — otherwise bit-flipped values eventually
    // fit under the threshold and corrupt the encoder accumulator.
    constexpr double MAX_DT_S          = 0.200;
    constexpr auto   LOOP_PERIOD       = std::chrono::milliseconds(20);

    // Acceleration limits — robot ramps toward set_velocity() target at these rates.
    // Baseline values scale with ROBOT_SPEED so the whole motion stack
    // (velocity caps + accel ramps) speeds up or slows down together.
    constexpr double ACCEL_V_MM_S2     = 150.0 * ROBOT_SPEED;  // forward accel
    constexpr double ACCEL_W_RAD_S2    = 1.5   * ROBOT_SPEED;  // yaw accel
    constexpr double DT_S              = std::chrono::duration<double>(LOOP_PERIOD).count();
    constexpr double V_STEP_PER_TICK   = ACCEL_V_MM_S2  * DT_S;
    constexpr double W_STEP_PER_TICK   = ACCEL_W_RAD_S2 * DT_S;
}

constexpr double Motor::TICKS_PER_MM;
constexpr double Motor::WHEELBASE_MM;

namespace {
    // Arduino SPI speed unit is "100 pulses/sec".
    //   pulses_per_sec = mm/s * TICKS_PER_MM
    //   pps100         = pulses_per_sec / 100 = mm/s * TICKS_PER_MM / 100
    constexpr double PPS100_PER_MM_S = Motor::TICKS_PER_MM / 100.0;

    // Chassis-level reach derived from the per-wheel pps ceiling.
    // Used to clamp _v_target/_w_target and the integrator so a stuck-high
    // command can't wind _v_cur up past what the wheels can physically do.
    constexpr double MAX_V_MM_S  = MAX_CMD_PPS100 / PPS100_PER_MM_S;
    constexpr double MAX_W_RAD_S = 2.0 * MAX_V_MM_S / Motor::WHEELBASE_MM;

    inline int16_t mm_s_to_pps100(double mm_s) {
        // Hard per-wheel pps ceiling — MAX_CMD_PPS100 (~680) is far below
        // int16 range, so this clamp also keeps the cast well-defined.
        long rounded = std::lround(mm_s * PPS100_PER_MM_S);
        const long cap = static_cast<long>(MAX_CMD_PPS100);
        if (rounded >  cap) rounded =  cap;
        if (rounded < -cap) rounded = -cap;
        return static_cast<int16_t>(rounded);
    }

}

Motor::Motor() = default;

Motor::~Motor() {
    stop();
}

bool Motor::start() {
    if (_motor_running.load()) return true;
    if (wiringPiSPISetup(1, 1000000) < 0) {
          LOGE("Motor") << "wiringPiSPISetup(1) failed";
          return false;
      }
    MotorDataType md = {0, 0, 0, 0, 0, 0};
    Send_Read_Motor_Data(&md);
    const int32_t seed_l = md.Encoder_M1;
    const int32_t seed_r = md.Encoder_M2;

    {
        std::lock_guard<std::mutex> lock(_mutex_motor);
        _last_acc_left  = seed_l;
        _last_acc_right = seed_r;
        _last_acc_ts    = std::chrono::steady_clock::now();
        _v_target = 0.0; _w_target = 0.0;
        _v_cur    = 0.0; _w_cur    = 0.0;
    }
    _motor_running.store(true);
    _io = std::thread(&Motor::_loop_motor_read_write, this);
    LOG("Motor") << "Started Motor with seed encoders: ("
                 << seed_l << ", " << seed_r
                 << ") TICKS_PER_MM=" << TICKS_PER_MM
                 << " WHEELBASE_MM=" << WHEELBASE_MM;
    return true;
}

void Motor::stop() {
    if (!_motor_running.load()) return;
    set_velocity(0.0, 0.0);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(1000));
    if (!_motor_running.exchange(false)) return;
    if (_io.joinable()) _io.join();
    // cleaning up motor io
    MotorDataType md = {0, 0, 0, 0, 0, 0};
    Send_Read_Motor_Data(&md);
    LOG("Motor") << "stopped";
}

void Motor::set_velocity(double v_mm_s, double omega_rad_s) {
    if (!std::isfinite(v_mm_s) || !std::isfinite(omega_rad_s)) {
        LOGE("Motor") << "set_velocity rejected non-finite input: v="
                      << v_mm_s << " w=" << omega_rad_s;
        return;
    }
    std::lock_guard<std::mutex> lock(_mutex_motor);
    _v_target = std::max(-MAX_V_MM_S,  std::min(v_mm_s,      MAX_V_MM_S));
    _w_target = std::max(-MAX_W_RAD_S, std::min(omega_rad_s, MAX_W_RAD_S));
}

void Motor::get_current_v_w(double& v_mm_s, double& omega_rad_s) const {
    std::lock_guard<std::mutex> lock(_mutex_motor);
    v_mm_s      = _v_cur;
    omega_rad_s = _w_cur;
}

void Motor::get_encoders(double& left_mm, double& right_mm) const {
    std::lock_guard<std::mutex> lock(_mutex_motor);
    const int64_t l_ticks =   _enc_left_acc;
    const int64_t r_ticks = -_enc_right_acc;
    left_mm  = static_cast<double>(l_ticks) / TICKS_PER_MM;
    right_mm = static_cast<double>(r_ticks) / TICKS_PER_MM;
}

void Motor::_loop_motor_read_write() {
    struct ZeroFrameOnExit {
        // always send 0 when app is forced quit
        ~ZeroFrameOnExit() {
            MotorDataType md = {0, 0, 0, 0, 0, 0};
            Send_Read_Motor_Data(&md);
        }
    } zero_on_exit;
    while (_motor_running.load() && !app::shutdown) {
        // for smoother v changes
        int32_t last_l, last_r;
        std::chrono::steady_clock::time_point last_ts;
        int16_t set_l, set_r;
        {
            std::lock_guard<std::mutex> lock(_mutex_motor);

            // Nudge current command toward target by at most one accel step per tick.
            _v_cur += std::max(-V_STEP_PER_TICK, std::min(_v_target - _v_cur, V_STEP_PER_TICK));
            _w_cur += std::max(-W_STEP_PER_TICK, std::min(_w_target - _w_cur, W_STEP_PER_TICK));
            // Anti-windup: keep the integrator within physical chassis reach.
            _v_cur = std::max(-MAX_V_MM_S,  std::min(_v_cur, MAX_V_MM_S));
            _w_cur = std::max(-MAX_W_RAD_S, std::min(_w_cur, MAX_W_RAD_S));

            // Diff-drive IK; right wheel is wired backwards, so invert its sign.
            const double half_b = 0.5 * WHEELBASE_MM;
            set_l =  mm_s_to_pps100(_v_cur - _w_cur * half_b);
            set_r = -mm_s_to_pps100(_v_cur + _w_cur * half_b);

            last_l  = _last_acc_left;
            last_r  = _last_acc_right;
            last_ts = _last_acc_ts;
        }

        auto now = std::chrono::steady_clock::now();
        double dt_s = std::chrono::duration<double>(now - last_ts).count();
        // Floor prevents the first cycle (or a paused thread) from yielding
        // an absurdly small threshold; ceiling prevents max_delta from
        // ballooning across sustained bad reads.
        if (dt_s < 0.001)    dt_s = 0.001;
        if (dt_s > MAX_DT_S) dt_s = MAX_DT_S;
        const int32_t max_delta = static_cast<int32_t>(
            MAX_TICKS_PER_SEC * dt_s * SAFETY_FACTOR);

        MotorDataType md = {0, 0, 0, 0, 0, 0};
        md.Set_Speed_M1 = set_l;
        md.Set_Speed_M2 = set_r;
        Send_Read_Motor_Data(&md);
        const int32_t last_dl = md.Encoder_M1 - last_l;
        const int32_t last_dr = md.Encoder_M2 - last_r;
        const bool accepted = (std::abs(last_dl) <= max_delta &&
                               std::abs(last_dr) <= max_delta);

        if (accepted) {
            auto accept_ts = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lock(_mutex_motor);
            // last_dl/last_dr were computed as a signed int32 subtraction
            // (md.Encoder_M{1,2} - last_{l,r}); under two's-complement
            // arithmetic this is the correct signed delta even when the
            // raw int32 encoder counter wrapped between reads. Promoting
            // to int64 and accumulating gives a cumulative travel that
            // does not overflow until ~58000 years at 1 m/s.
            _enc_left_acc  += static_cast<int64_t>(last_dl);
            _enc_right_acc += static_cast<int64_t>(last_dr);
            _last_acc_left  = md.Encoder_M1;
            _last_acc_right = md.Encoder_M2;
            _last_acc_ts    = accept_ts;
        } else {
            std::lock_guard<std::mutex> lock(_mutex_motor);
            ++_spi_reject_count;
            LOGE("Motor") << "SPI bad read (dL=" << last_dl
                          << " dR=" << last_dr
                          << " max=" << max_delta
                          << "); holding previous encoder values.";
        }
        std::this_thread::sleep_for(LOOP_PERIOD);
    }
}

void Motor::get_spi_stats(uint64_t& rejects) const {
    std::lock_guard<std::mutex> lock(_mutex_motor);
    rejects = _spi_reject_count;
}

