#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

// Low-level: write target angle to PWM channel. Prefer the helpers below
// for gripper control — they move smoothly between open and closed angles.
void Servo(int ch, int deg); // ch range [0,1]. deg range [0,180]

// Smooth gripper control. Blocking; tracks per-channel angle internally so
// repeated calls are no-ops once the target position is reached.
void servo_grab();
void servo_release();

#ifdef __cplusplus
}
#endif

#endif