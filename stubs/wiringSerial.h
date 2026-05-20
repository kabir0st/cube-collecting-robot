#pragma once

#ifdef __cplusplus
extern "C" {
#endif

int  serialOpen(const char* device, const int baud);
void serialClose(const int fd);
void serialFlush(const int fd);
void serialPutchar(const int fd, const unsigned char c);
void serialPuts(const int fd, const char* s);
int  serialDataAvail(const int fd);
int  serialGetchar(const int fd);

#ifdef __cplusplus
}
#endif
