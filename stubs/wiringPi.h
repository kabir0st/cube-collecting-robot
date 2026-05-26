#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define PWM_OUTPUT  2
#define PWM_MODE_MS 0

void          delay(unsigned int howLong);
void          delayMicroseconds(unsigned int howLong);
unsigned int  millis(void);
unsigned int  micros(void);
int  wiringPiSetup(void);
int  wiringPiSetupGpio(void);
void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
int  digitalRead(int pin);
void pwmWrite(int pin, int value);
void pwmSetMode(int mode);
void pwmSetRange(unsigned int range);
void pwmSetClock(int divisor);

#ifdef __cplusplus
}
#endif
