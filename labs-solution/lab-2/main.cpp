#include <iostream>
#include <thread>
#include <fstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <server.hpp>
#include <exception>
#include <stdexcept>
#include <chrono>


using namespace std;

const int LISTEN_FOR = 10;

int main() {
    std::exception_ptr error;
    cout << "[info] Starting Data Listen Server . . " << endl;
    // imported global server from server.hpp to user atomic variable
    // LidarDataServer server = LidarDataServer();
    std::thread server_thread([&error]() {
        try {
            if (server.start_stream() != 0) {
                throw std::runtime_error("lidar stream exited with error");
            }
        } catch (const std::exception& e) {
            std::cerr << "[error] " << e.what() << std::endl;
            error = std::current_exception();
            server.stop_lidar();
        } catch (...) {
            std::cerr << "[error] unknown stream exception" << std::endl;
            error = std::current_exception();
            server.stop_lidar();
        }
    });
    cout << "[info] Waiting for lister to be ready." << endl;
    // waiting for the server to be ready
    while (!server.is_ready()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    cout << "[info] Starting lidar now" << endl;
    try {
        if (server.start_lidar() != 0) {
            throw std::runtime_error("failed to start lidar");
        }
        cout << "[info] Sleeping for a bit . . . " << endl;
        std::this_thread::sleep_for(
            std::chrono::seconds(LISTEN_FOR));
    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << std::endl;
    }
    server.stop_lidar();
    server_thread.join();
    if (error) {
        try {
            std::rethrow_exception(error);
        } catch (const std::exception& e) {
            std::cerr << "[error] " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[error] unknown exception" << std::endl;
        }
    }
    return 0;
}
