#include <motor.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
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
    // Hard ceiling on commanded per-wheel pps sent to the Arduino. At
    // ~119 ticks/mm this caps wheel-frame speed at ~504 mm/s — roughly a
    // small cat's walking pace — with chassis ω-cap headroom of
    // 2.0 · 504 / 185 ≈ 5.45 rad/s. The motor nameplate is ~2660 mm/s at
    // the shaft, so this is still well inside the physical envelope, but
    // beyond ~500 mm/s the chassis starts exhibiting noticeable battery
    // sag and wheel-slip on smooth surfaces. Don't push past ~720 without
    // revisiting those.
    constexpr double MAX_CMD_PPS100    = 600.0;
    // jitter margin
    constexpr double SAFETY_FACTOR     = 2.0;
    // Cap on the time window used for max_delta. Prevents the per-tick
    // physical-travel threshold from growing without bound across a
    // burst of bad SPI reads — otherwise bit-flipped values eventually
    // fit under the threshold and corrupt the encoder accumulator.
    constexpr double MAX_DT_S          = 0.200;
    constexpr auto   LOOP_PERIOD       = std::chrono::milliseconds(20);

    // Acceleration baselines (per second). The actual per-tick step is
    // computed inside the motor IO loop using the runtime ROBOT_SPEED,
    // because ROBOT_SPEED is set from config in main() and these constants
    // would otherwise capture the static-init default before the config
    // load runs.
    constexpr double ACCEL_V_BASE_MM_S2  = 150.0;  // forward accel @ ROBOT_SPEED=1
    constexpr double ACCEL_W_BASE_RAD_S2 =   1.5;  // yaw accel     @ ROBOT_SPEED=1
    constexpr double DT_S = std::chrono::duration<double>(LOOP_PERIOD).count();
}

constexpr double Motor::LEFT_TICKS_PER_MM;
constexpr double Motor::RIGHT_TICKS_PER_MM;
constexpr double Motor::WHEELBASE_MM;

namespace {
    // Arduino SPI speed unit is "100 pulses/sec".
    //   pulses_per_sec = mm/s * ticks_per_mm
    //   pps100         = pulses_per_sec / 100
    // Per-wheel so a 2% wheel asymmetry doesn't bias either the commanded
    // velocity or the resulting heading.
    constexpr double PPS100_PER_MM_S_LEFT  = Motor::LEFT_TICKS_PER_MM  / 100.0;
    constexpr double PPS100_PER_MM_S_RIGHT = Motor::RIGHT_TICKS_PER_MM / 100.0;

    // Chassis-level reach derived from the per-wheel pps ceiling. Use the
    // slower wheel (larger ticks/mm) so neither wheel ever saturates: when
    // commanded at MAX_V_MM_S, the slow wheel sits exactly at MAX_CMD_PPS100
    // and the fast wheel has headroom.
    constexpr double SLOWER_PPS100_PER_MM_S =
        (PPS100_PER_MM_S_LEFT > PPS100_PER_MM_S_RIGHT)
            ? PPS100_PER_MM_S_LEFT : PPS100_PER_MM_S_RIGHT;
    constexpr double MAX_V_MM_S  = MAX_CMD_PPS100 / SLOWER_PPS100_PER_MM_S;
    constexpr double MAX_W_RAD_S = 2.0 * MAX_V_MM_S / Motor::WHEELBASE_MM;

    inline int16_t mm_s_to_pps100(double mm_s, double pps100_per_mm_s) {
        // Hard per-wheel pps ceiling — MAX_CMD_PPS100 (~680) is far below
        // int16 range, so this clamp also keeps the cast well-defined.
        long rounded = std::lround(mm_s * pps100_per_mm_s);
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

    // Settle pause after SPI init. The bus lines need a few ms to come
    // out of their pre-init state; sending commands immediately can
    // clock garbage bytes into the Arduino that look like asymmetric
    // speed commands and rotate the chassis a few degrees.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Hammer zero frames to fully overwrite any stale Arduino state
    // (e.g., previous run was SIGKILL'd while driving). One frame is
    // not always enough — the controller may finish its current PWM
    // cycle at the previous speed before applying the new command, so
    // we send N frames at the motor IO loop period to cover several
    // PWM cycles before doing anything else.
    //
    // memset (not aggregate init) so padding bytes in MotorDataType are
    // guaranteed zero — otherwise an Arduino that parses by struct
    // offset could see stack garbage in padding slots.
    MotorDataType md;
    constexpr int   ZERO_FRAMES = 8;
    constexpr auto  ZERO_GAP    = std::chrono::milliseconds(25);  // 8 × 25ms = 200ms
    for (int i = 0; i < ZERO_FRAMES; ++i) {
        std::memset(&md, 0, sizeof(md));
        Send_Read_Motor_Data(&md);
        std::this_thread::sleep_for(ZERO_GAP);
    }

    // Poll encoders until the wheels are actually stationary. If a
    // previous run left them spinning, the zero frames have now arrived
    // but inertia can keep them coasting for a few hundred ms more.
    // Two successive reads with negligible delta = at rest. Cap the
    // wait so a stuck-spinning chassis doesn't deadlock startup.
    constexpr auto  POLL_GAP        = std::chrono::milliseconds(50);
    constexpr auto  POLL_TIMEOUT    = std::chrono::seconds(3);
    constexpr int32_t REST_TICK_TOL = 2;   // ~17 µm at 119 ticks/mm — sensor noise
    const auto t_poll_start = std::chrono::steady_clock::now();
    int32_t prev_l = 0, prev_r = 0;
    bool first = true;
    while (true) {
        std::memset(&md, 0, sizeof(md));
        Send_Read_Motor_Data(&md);
        if (!first) {
            const int32_t dl = std::abs(md.Encoder_M1 - prev_l);
            const int32_t dr = std::abs(md.Encoder_M2 - prev_r);
            if (dl <= REST_TICK_TOL && dr <= REST_TICK_TOL) break;
        }
        prev_l = md.Encoder_M1;
        prev_r = md.Encoder_M2;
        first = false;
        if (std::chrono::steady_clock::now() - t_poll_start > POLL_TIMEOUT) {
            LOGE("Motor") << "Wheels still moving after "
                          << POLL_TIMEOUT.count()
                          << "s of zero frames — proceeding anyway.";
            break;
        }
        std::this_thread::sleep_for(POLL_GAP);
    }

    // Final read for the encoder seed in the settled state.
    std::memset(&md, 0, sizeof(md));
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
                 << ") LEFT_TICKS_PER_MM=" << LEFT_TICKS_PER_MM
                 << " RIGHT_TICKS_PER_MM=" << RIGHT_TICKS_PER_MM
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
    left_mm  = static_cast<double>(l_ticks) / LEFT_TICKS_PER_MM;
    right_mm = static_cast<double>(r_ticks) / RIGHT_TICKS_PER_MM;
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
        // Per-tick accel step is recomputed from the runtime ROBOT_SPEED so a
        // mid-run tweak (or the config-driven boot value) propagates without
        // needing to recompile.
        const double v_step = ACCEL_V_BASE_MM_S2  * ROBOT_SPEED * DT_S;
        const double w_step = ACCEL_W_BASE_RAD_S2 * ROBOT_SPEED * DT_S;
        {
            std::lock_guard<std::mutex> lock(_mutex_motor);

            // Nudge current command toward target by at most one accel step per tick.
            _v_cur += std::max(-v_step, std::min(_v_target - _v_cur, v_step));
            _w_cur += std::max(-w_step, std::min(_w_target - _w_cur, w_step));
            // Anti-windup: keep the integrator within physical chassis reach.
            _v_cur = std::max(-MAX_V_MM_S,  std::min(_v_cur, MAX_V_MM_S));
            _w_cur = std::max(-MAX_W_RAD_S, std::min(_w_cur, MAX_W_RAD_S));

            // Diff-drive IK; right wheel is wired backwards, so invert its sign.
            const double half_b = 0.5 * WHEELBASE_MM;
            set_l =  mm_s_to_pps100(_v_cur - _w_cur * half_b,
                                    PPS100_PER_MM_S_LEFT);
            set_r = -mm_s_to_pps100(_v_cur + _w_cur * half_b,
                                    PPS100_PER_MM_S_RIGHT);

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

