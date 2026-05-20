#include <motor.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <thread>
#include <global.hpp>
#include <loggers.hpp>
#include <spi_com.h>
#include <wiringPiSPI.h>

namespace {
    // Physical ceiling on encoder ticks/sec for either wheel.
    // Nominal motor 4670 rpm / 6.6 gear * 4096 quad ticks/rev ≈ 319,000 ticks/s.
    // 400,000 sits comfortably above that.
    constexpr double MAX_TICKS_PER_SEC = 400000.0;
    // Hard ceiling on commanded per-wheel pps sent to the Arduino.
    // 17% of the physical max ≈ 68,000 pps ≈ 400 mm/s — leaves ~54%
    // headroom over the controller's worst-case wheel-frame speed
    // (~260 mm/s from v=75 mm/s + omega·b/2) so a runaway command can
    // never ship more than ~1.5x normal operating speed.
    constexpr double MAX_CMD_PPS100    = 0.17 * MAX_TICKS_PER_SEC / 100.0;
    // jitter margin
    constexpr double SAFETY_FACTOR     = 2.0;
    constexpr int    MAX_RETRIES       = 4;
    constexpr auto   LOOP_PERIOD       = std::chrono::milliseconds(20);

    // Acceleration limits — robot ramps toward set_velocity() target at these rates.
    // Baseline values scale with ROBOT_SPEED so the whole motion stack
    // (velocity caps + accel ramps) speeds up or slows down together.
    constexpr double ACCEL_V_MM_S2     = 150.0 * ROBOT_SPEED;  // forward accel
    constexpr double ACCEL_W_RAD_S2    = 1.5   * ROBOT_SPEED;  // yaw accel
    constexpr double DT_S              = 0.020;  // matches LOOP_PERIOD
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

    inline int16_t mm_s_to_pps100(double mm_s) {
        // Round first, then clamp into int16 range. The original
        //     clamp -> add 0.5 -> cast
        // overflowed the int16 cast for inputs at the very negative end
        // (e.g. pps100 = -32768.0 → -32768.5 → cast is UB), and for
        // positive overflow above 32767 → 32767.5 → cast also UB. Using
        // std::lround gives a well-defined integer result that we then
        // clamp with integer arithmetic.
        long rounded = std::lround(mm_s * PPS100_PER_MM_S);
        const long cap = static_cast<long>(MAX_CMD_PPS100);
        if (rounded >  cap) rounded =  cap;
        if (rounded < -cap) rounded = -cap;
        if (rounded >  32767L) rounded =  32767L;
        if (rounded < -32768L) rounded = -32768L;
        return static_cast<int16_t>(rounded);
    }

    inline double pps100_to_mm_s(int16_t pps100) {
        return static_cast<double>(pps100) / PPS100_PER_MM_S;
    }
}

Motor::Motor() = default;

Motor::~Motor() {
    stop();
}

bool Motor::start() {
    if (_motor_running.load()) return true;
    if (wiringPiSPISetup(1, 4000000) < 0) {
          LOGE("Motor") << "wiringPiSPISetup(1) failed";
          return false;
      }
    // Bumpless transfer: the Arduino motor controller may be holding a
    // non-zero Set_Speed across Pi reboots. Slamming Set_Speed to 0 on the
    // first frame would brake hard against any standing PWM and visibly
    // twitch the wheels. Instead, probe with one zero-set frame, read the
    // Arduino's reported Act_Speed, then mirror it back on the remaining
    // seed frames so the controller keeps its current PWM. Initialize
    // _v_cur/_w_cur from the measured speed and let the IO-loop ramp
    // decelerate to zero at the configured ACCEL limits.
    std::array<int32_t, 3> seed_l{0,0,0}, seed_r{0,0,0};
    MotorDataType md = {0, 0, 0, 0, 0, 0};
    int16_t hold_l = 0, hold_r = 0;
    for (int i = 0; i < 3; ++i) {
        md.Set_Speed_M1 = hold_l;
        md.Set_Speed_M2 = hold_r;
        Send_Read_Motor_Data(&md);
        seed_l[i] = md.Encoder_M1;
        seed_r[i] = md.Encoder_M2;
        if (i == 0) {
            // After the first probe we know what the Arduino is doing —
            // mirror it back on frames #2 and #3 so it doesn't decelerate
            // between our reads.
            hold_l = md.Act_Speed_M1;
            hold_r = md.Act_Speed_M2;
        }
    }
    std::sort(seed_l.begin(), seed_l.end());
    std::sort(seed_r.begin(), seed_r.end());

    // Invert IK from _loop_motor_read_write: right wheel is wired
    // backwards, so its sign flips when going chassis-frame ↔ wheel-frame.
    const double act_l_mm_s =  pps100_to_mm_s(md.Act_Speed_M1);
    const double act_r_mm_s = -pps100_to_mm_s(md.Act_Speed_M2);
    const double v_seed = 0.5 * (act_l_mm_s + act_r_mm_s);
    const double w_seed = (act_r_mm_s - act_l_mm_s) / WHEELBASE_MM;

    {
        std::lock_guard<std::mutex> lock(_mutex_motor);
        _enc_left      = seed_l[1];
        _enc_right     = seed_r[1];
        _last_acc_left  = seed_l[1];
        _last_acc_right = seed_r[1];
        _last_acc_ts    = std::chrono::steady_clock::now();
        _act_left  = md.Act_Speed_M1;
        _act_right = md.Act_Speed_M2;
        // Smoother seeded from the wheels' actual current speed; target
        // stays at zero so the IO loop ramps it down at ACCEL_V_MM_S2 /
        // ACCEL_W_RAD_S2 instead of slamming Set_Speed to 0.
        _v_target = 0.0;    _w_target = 0.0;
        _v_cur    = v_seed; _w_cur    = w_seed;
    }
    _motor_running.store(true);
    _io = std::thread(&Motor::_loop_motor_read_write, this);
    LOG("Motor") << "Started Motor with seed encoders: ("
                 << seed_l[1] << ", " << seed_r[1] << ")"
                 << " boot Act_Speed pps100 L=" << md.Act_Speed_M1
                 << " R=" << md.Act_Speed_M2
                 << " → v_cur=" << v_seed << " mm/s"
                 << " w_cur=" << w_seed << " rad/s";
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
    std::lock_guard<std::mutex> lock(_mutex_motor);
    _v_target = v_mm_s;
    _w_target = omega_rad_s;
}

void Motor::get_current_v_w(double& v_mm_s, double& omega_rad_s) const {
    std::lock_guard<std::mutex> lock(_mutex_motor);
    v_mm_s      = _v_cur;
    omega_rad_s = _w_cur;
}

void Motor::get_encoders(double& left_mm, double& right_mm) const {
    std::lock_guard<std::mutex> lock(_mutex_motor);
    const int64_t l_ticks =   _enc_left_acc  - _enc_left_noise;
    const int64_t r_ticks = -(_enc_right_acc - _enc_right_noise);
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

            // Diff-drive IK; right wheel is wired backwards, so invert its sign.
            const double half_b = 0.5 * WHEELBASE_MM;
            _set_left  =  mm_s_to_pps100(_v_cur - _w_cur * half_b);
            _set_right = -mm_s_to_pps100(_v_cur + _w_cur * half_b);

            last_l  = _last_acc_left;
            last_r  = _last_acc_right;
            last_ts = _last_acc_ts;
            set_l   = _set_left;
            set_r   = _set_right;
        }

        auto now = std::chrono::steady_clock::now();
        double dt_s = std::chrono::duration<double>(now - last_ts).count();
        // Clamp dt to a sane floor so the first cycle (or a paused thread)
        // doesn't yield an absurdly small threshold.
        if (dt_s < 0.001) dt_s = 0.001;
        const int32_t max_delta = static_cast<int32_t>(
            MAX_TICKS_PER_SEC * dt_s * SAFETY_FACTOR);

        MotorDataType md = {0, 0, 0, 0, 0, 0};
        bool accepted = false;
        int32_t last_dl = 0, last_dr = 0;
        for (int attempt = 0; attempt < MAX_RETRIES; ++attempt) {
            md.Set_Speed_M1 = set_l;
            md.Set_Speed_M2 = set_r;
            Send_Read_Motor_Data(&md);
            last_dl = md.Encoder_M1 - last_l;
            last_dr = md.Encoder_M2 - last_r;
            if (std::abs(last_dl) <= max_delta &&
                std::abs(last_dr) <= max_delta) {
                accepted = true;
                break;
            }
            // got bad read count it and re-issue immediately
            std::lock_guard<std::mutex> lock(_mutex_motor);
            ++_spi_reject_count;
        }

        if (accepted) {
            auto accept_ts = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lock(_mutex_motor);
            _act_left       = md.Act_Speed_M1;
            _act_right      = md.Act_Speed_M2;
            _enc_left       = md.Encoder_M1;
            _enc_right      = md.Encoder_M2;
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
            {
                std::lock_guard<std::mutex> lock(_mutex_motor);
                ++_spi_give_up_count;
            }
            LOGE("Motor") << "SPI giveup after " << MAX_RETRIES
                          << " bad reads (last dL=" << last_dl
                          << " dR=" << last_dr
                          << " max=" << max_delta
                          << "); holding previous encoder values.";
        }
        std::this_thread::sleep_for(LOOP_PERIOD);
    }
}

void Motor::get_spi_stats(uint64_t& rejects, uint64_t& give_ups) const {
    std::lock_guard<std::mutex> lock(_mutex_motor);
    rejects = _spi_reject_count;
    give_ups = _spi_give_up_count;
}

