#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <types.hpp>
#include <thread>
#include <future>
#include <calibrate.hpp>
#include <server.hpp>
#include <global.hpp>
#include <iostream>
#include <chrono>
#include <utils.hpp>

int send_control_signal(int start) {
    int sock = -1;
    try {
        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            throw std::runtime_error("failed to create control socket");
        }

        struct sockaddr_in server = {};
        server.sin_family = AF_INET;
        server.sin_port = htons(CONTORL_PORT);
        if (inet_pton(AF_INET, TCP_IP, &server.sin_addr) <= 0) {
            throw std::runtime_error("failed to parse control server IP");
        }

        if (connect(sock, (struct sockaddr*)&server, sizeof(server)) < 0) {
            throw std::runtime_error("failed to connect to control socket");
        }

        const char data[] = {start ? (char)0x10 : (char)0x20, 0x00};
        if (send(sock, data, 2, 0) != 2) {
            throw std::runtime_error("failed to send control signal");
        }

        std::cout << "[info] Control signal sent" << std::endl;
        close(sock);
        return 0;
    } catch (const std::exception& e) {
        if (sock >= 0) {
            close(sock);
            
        }
        std::cerr << "[error] " << e.what() << std::endl;
        return -1;
    }
}



ListenerThread::~ListenerThread() {
    if (thread.joinable()) {
        thread.join();
    }
}
