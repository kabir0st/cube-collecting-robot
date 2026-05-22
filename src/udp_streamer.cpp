#include <udp_streamer.hpp>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstdio>
#include <cmath>

UdpStreamer::UdpStreamer(const char *dest_ip, int port)
{
    _sock = socket(AF_INET, SOCK_DGRAM, 0);
    _dest.sin_family = AF_INET;
    _dest.sin_port = htons(static_cast<uint16_t>(port));
    inet_pton(AF_INET, dest_ip, &_dest.sin_addr);
}

UdpStreamer::~UdpStreamer()
{
    if (_sock >= 0)
        close(_sock);
}

void UdpStreamer::send(double time_s,
                       const Pos2D &pose, const Pose2DCov &cov,
                       double d_mm, double da_deg, int source)
{
    // Fields 1-5  : same layout as test_udp_sender → MATLAB plotter works unchanged
    // Fields 6-12 : time_s + 6 unique values of symmetric 3x3 cov (row-major)
    // Field  13   : source id (0=K, 1=O, 2=S)
    char buf[256];
    int len = std::snprintf(buf, sizeof(buf),
                            "%.3f,%.3f,%.4f,%.4f,%.4f,"      // x_mm, y_mm, theta_deg, d_mm, da_deg
                            "%.3f,"                          // time_s
                            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f," // cov: xx xy xt yy yt tt
                            "%d\n",                          // source id
                            pose.x, pose.y,
                            pose.theta * (180.0 / M_PI),
                            d_mm, da_deg,
                            time_s,
                            cov.xx, cov.xy, cov.xt,
                            cov.yy, cov.yt, cov.tt,
                            source);

    ::sendto(_sock, buf, len, 0,
             reinterpret_cast<sockaddr *>(&_dest), sizeof(_dest));
}
