#pragma once
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

void delay(unsigned int howLong);
void delayMicroseconds(unsigned int howLong);
int  wiringPiSetup(void);
int  wiringPiSetupGpio(void);
void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
int  digitalRead(int pin);

#ifdef __cplusplus
}
#endif
