#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

// Non-blocking: latch a target angle for the channel. The actual slew is
// driven by servo_tick() to keep peak current bounded (avoids the 5 V
// brownout that a single large jump causes on the Pi). ch in [0,1],
// deg in [0,180].
void Servo(int ch, int deg);

// Non-blocking gripper helpers — latch CLOSED_DEG / OPEN_DEG as the target.
void servo_grab();
void servo_release();

// Call once per control-loop iteration. Advances each channel one STEP_DEG
// toward its latched target if STEP_MS has elapsed since the last advance.
// Safe to call at any rate ≥ 50 Hz; faster ticks are paced internally.
void servo_tick(void);

// True once the channel's current angle has reached the latched target.
int servo_done(int ch);

#ifdef __cplusplus
}
#endif

#endif