// test_udp_sender.cpp
// Standalone UDP sender — simulates robot odometry packets for MATLAB testing.
// Nothing from the robot codebase is used here.
//
// Compile (MinGW): g++ -std=c++14 -o test_sender test_udp_sender.cpp -lws2_32
// Run:             test_sender.exe <YOUR_PC_IP>   (use 127.0.0.1 for localhost)

#include <winsock2.h>
#include <ws2tcpip.h>              // for inet_pton on Windows
#pragma comment(lib, "ws2_32.lib") // only needed for MSVC; MinGW uses -l flag
#include <cstdio>
#define _USE_MATH_DEFINES
#include <cmath>
#include <chrono>
#include <thread>

int main(int argc, char *argv[])
{
    const char *pc_ip = (argc > 1) ? argv[1] : "192.168.X.X";
    const int port = 5005;
    const double dt = 0.04; // 40 ms per cycle — same rate as real loop_control

    // --- Winsock init ---
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);

    // --- UDP socket ---
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in dest{};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(port);
    inet_pton(AF_INET, pc_ip, &dest.sin_addr);

    // --- Simulate: circle of radius 500 mm centered at (1000, 1000) ---
    const double R = 500.0;     // mm
    const double v = 150.0;     // mm/s  (matches real robot speed)
    const double omega = v / R; // rad/s to stay on circle

    double x = 1000.0 + R; // start on the right of center
    double y = 1000.0;
    double theta = M_PI / 2.0; // facing upward
    int pkt = 0;

    printf("Sending to %s:%d at 25 Hz — Ctrl-C to stop\n", pc_ip, port);

    while (true)
    {
        // incremental motion this cycle
        double d_mm = v * dt;
        double d_th = omega * dt;

        // midpoint-heading integration (same model as KF predict in kalman.cpp)
        double th_m = theta + 0.5 * d_th;
        x += d_mm * std::cos(th_m);
        y += d_mm * std::sin(th_m);
        theta += d_th;

        // wrap theta to (-pi, pi]
        while (theta > M_PI)
            theta -= 2.0 * M_PI;
        while (theta <= -M_PI)
            theta += 2.0 * M_PI;

        // format packet — same layout MATLAB expects
        char buf[128];
        int len = std::snprintf(buf, sizeof(buf),
                                "%.3f,%.3f,%.3f,%.3f,%.3f\n",
                                x,                    // x_mm
                                y,                    // y_mm
                                theta * 180.0 / M_PI, // theta_deg
                                d_mm,                 // d_mm  (increment)
                                d_th * 180.0 / M_PI); // da_deg (increment)

        sendto(sock, buf, len, 0,
               reinterpret_cast<sockaddr *>(&dest), sizeof(dest));

        pkt++;
        if (pkt % 25 == 0) // print status once per second
            printf("pkt=%4d  x=%7.1f  y=%7.1f  theta=%6.1f deg\n",
                   pkt, x, y, theta * 180.0 / M_PI);

        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }

    closesocket(sock);
    WSACleanup();
    return 0;
}
