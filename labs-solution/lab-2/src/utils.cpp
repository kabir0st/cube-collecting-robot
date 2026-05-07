#include <iostream>
#include <stdexcept>
#include <cstring>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <types.hpp>
#include <utils.hpp>
#include <log.hpp>


// One-shot TCP message to the lidar driver: 0x10 to start, 0x20 to stop,
// followed by a single zero byte. The driver acks by toggling its scan.
int send_control_signal(int start) {
    int sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        LOGE("ctrl") << "socket() failed";
        return -1;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(CONTROL_PORT);
    if (::inet_pton(AF_INET, TCP_IP, &addr.sin_addr) <= 0) {
        LOGE("ctrl") << "bad ip " << TCP_IP;
        ::close(sock);
        return -1;
    }

    if (::connect(sock, (sockaddr*)&addr, sizeof(addr)) < 0) {
        LOGE("ctrl") << "connect() to " << TCP_IP << ":" << CONTROL_PORT << " failed";
        ::close(sock);
        return -1;
    }

    const char data[2] = { start ? (char)0x10 : (char)0x20, 0x00 };
    if (::send(sock, data, sizeof(data), 0) != sizeof(data)) {
        LOGE("ctrl") << "send() failed";
        ::close(sock);
        return -1;
    }

    LOG("ctrl") << (start ? "START" : "STOP") << " sent";
    ::close(sock);
    return 0;
}
