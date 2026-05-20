#pragma once

#ifdef __cplusplus
extern "C" {
#endif

int wiringPiSPISetup(int channel, int speed);
int wiringPiSPISetupMode(int channel, int speed, int mode);
int wiringPiSPIDataRW(int channel, unsigned char* data, int len);

#ifdef __cplusplus
}
#endif
