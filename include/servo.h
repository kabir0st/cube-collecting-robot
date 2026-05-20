#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

void Servo(int ch, int deg); // ch defines channel range [0  1]. deg defines degree range [0  180]

#ifdef __cplusplus
}
#endif

#endif