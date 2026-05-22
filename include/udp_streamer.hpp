#pragma once
#include <sys/socket.h>
#include <netinet/in.h>
#include <types.hpp> // Pos2D, Pose2DCov

class UdpStreamer
{
public:
    // dest_ip : PC IP on the same LAN, e.g. "192.168.0.10"
    UdpStreamer(const char *dest_ip, int port = 5005);
    ~UdpStreamer();

    // source: 0=Kalman  1=Odometry  2=ScanMatch
    void send(double time_s,
              const Pos2D &pose, const Pose2DCov &cov,
              double d_mm, double da_deg,
              int source = 0);

private:
    int _sock = -1;
    sockaddr_in _dest{};
};