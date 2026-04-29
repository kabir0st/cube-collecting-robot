#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <stdexcept>
#include <chrono>
#include <thread>

#ifdef _WIN32
    #include <winsock2.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <sys/socket.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <netdb.h>
    #include <fcntl.h>
    #include <errno.h>
#endif

class TCPClient {
private:
    int sock;
    bool connected;
    
    // For Raspberry Pi: Set socket timeout
    bool setSocketTimeout(int timeout_ms) {
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        
        if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
            return false;
        if (setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0)
            return false;
        return true;
    }
    
public:
    TCPClient() : sock(-1), connected(false) {
#ifdef _WIN32
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            throw std::runtime_error("WSAStartup failed");
        }
#endif
    }
    
    ~TCPClient() {
        disconnect();
#ifdef _WIN32
        WSACleanup();
#endif
    }
    
    // Prevent copying
    TCPClient(const TCPClient&) = delete;
    TCPClient& operator=(const TCPClient&) = delete;
    
    // Move semantics
    TCPClient(TCPClient&& other) noexcept : sock(other.sock), connected(other.connected) {
        other.sock = -1;
        other.connected = false;
    }
    
    bool connect(const std::string& ip, int port, int timeout_ms = 5000) {
        // Create socket
        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            std::cerr << "Socket creation failed: " << strerror(errno) << std::endl;
            return false;
        }
        
        // Set socket options for Raspberry Pi optimization
        int opt = 1;
        setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        // Disable Nagle's algorithm for faster small packet sending (important for your use case)
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));
        
        // Set timeout
        if (!setSocketTimeout(timeout_ms)) {
            std::cerr << "Failed to set socket timeout" << std::endl;
        }
        
        // Prepare server address
        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        
        // Convert IP address
        if (inet_pton(AF_INET, ip.c_str(), &server_addr.sin_addr) <= 0) {
            std::cerr << "Invalid address: " << ip << std::endl;
            close(sock);
            sock = -1;
            return false;
        }
        
        // Connect with timeout
        // For non-blocking connect (better timeout control on ARM)
        fcntl(sock, F_SETFL, O_NONBLOCK);
        
        int result = ::connect(sock, (sockaddr*)&server_addr, sizeof(server_addr));
        if (result < 0) {
            if (errno == EINPROGRESS) {
                // Connection in progress, wait with select()
                fd_set fdset;
                FD_ZERO(&fdset);
                FD_SET(sock, &fdset);
                
                struct timeval tv;
                tv.tv_sec = timeout_ms / 1000;
                tv.tv_usec = (timeout_ms % 1000) * 1000;
                
                result = select(sock + 1, NULL, &fdset, NULL, &tv);
                if (result > 0) {
                    // Check if connection was successful
                    int error = 0;
                    socklen_t len = sizeof(error);
                    getsockopt(sock, SOL_SOCKET, SO_ERROR, &error, &len);
                    if (error != 0) {
                        std::cerr << "Connection failed: " << strerror(error) << std::endl;
                        close(sock);
                        sock = -1;
                        return false;
                    }
                } else {
                    std::cerr << "Connection timeout" << std::endl;
                    close(sock);
                    sock = -1;
                    return false;
                }
            } else {
                std::cerr << "Connection failed: " << strerror(errno) << std::endl;
                close(sock);
                sock = -1;
                return false;
            }
        }
        
        // Restore blocking mode
        int flags = fcntl(sock, F_GETFL, 0);
        fcntl(sock, F_SETFL, flags & ~O_NONBLOCK);
        
        connected = true;
        std::cout << "Connected to " << ip << ":" << port << std::endl;
        return true;
    }
    
    bool send(const std::vector<uint8_t>& data) {
        if (!connected || sock < 0) {
            std::cerr << "Not connected" << std::endl;
            return false;
        }
        
        ssize_t sent = ::send(sock, data.data(), data.size(), MSG_NOSIGNAL);
        if (sent < 0) {
            std::cerr << "Send failed: " << strerror(errno) << std::endl;
            return false;
        }
        
        if (static_cast<size_t>(sent) != data.size()) {
            // Handle partial send (unlikely with small packets, but safe)
            size_t remaining = data.size() - sent;
            while (remaining > 0) {
                sent = ::send(sock, data.data() + (data.size() - remaining), remaining, MSG_NOSIGNAL);
                if (sent < 0) {
                    std::cerr << "Partial send failed: " << strerror(errno) << std::endl;
                    return false;
                }
                remaining -= sent;
            }
        }
        
        return true;
    }
    
    // Convenience method for your specific use case
    bool sendRawBytes(const uint8_t* bytes, size_t length) {
        if (!connected || sock < 0) {
            std::cerr << "Not connected" << std::endl;
            return false;
        }
        
        ssize_t sent = ::send(sock, bytes, length, MSG_NOSIGNAL);
        if (sent < 0 || static_cast<size_t>(sent) != length) {
            std::cerr << "Send failed: " << strerror(errno) << std::endl;
            return false;
        }
        
        return true;
    }
    
    std::vector<uint8_t> receive(size_t buffer_size = 1024, int timeout_ms = 1000) {
        std::vector<uint8_t> buffer(buffer_size);
        
        if (!connected || sock < 0) {
            return {};
        }
        
        // Set receive timeout for this call
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        
        ssize_t received = ::recv(sock, buffer.data(), buffer.size(), 0);
        if (received < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Timeout
                return {};
            }
            std::cerr << "Receive failed: " << strerror(errno) << std::endl;
            connected = false;
            return {};
        }
        
        buffer.resize(received);
        return buffer;
    }
    
    void disconnect() {
        if (sock >= 0) {
            shutdown(sock, SHUT_RDWR);
            close(sock);
            sock = -1;
        }
        connected = false;
    }
    
    bool isConnected() const {
        return connected;
    }
};

// Example usage matching your Python code
int main() {
    try {
        TCPClient client;
        
        if (client.connect("127.0.0.1", 9887)) {
            // Your exact Python equivalent: send '\x10\x00'
            std::vector<uint8_t> data = {0x10, 0x00};
            // Or use: uint8_t data[] = {0x10, 0x00}; client.sendRawBytes(data, 2);
            
            if (client.send(data)) {
                std::cout << "Successfully sent: 0x10 0x00" << std::endl;
                
                // Optional: Wait for response
                auto response = client.receive(1024, 2000);
                if (!response.empty()) {
                    std::cout << "Received " << response.size() << " bytes" << std::endl;
                    for (auto byte : response) {
                        printf("0x%02X ", byte);
                    }
                    std::cout << std::endl;
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}