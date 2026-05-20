#pragma once
#include <iostream>
#include <sstream>
#include <thread>
#include <mutex>
#include <unistd.h>

// LOG("tag") << "message" prints `[tag] message\n` atomically.
// One global mutex handles stdout/stderr 

namespace logging_detail {
inline std::mutex& cout_mutex() {
    static std::mutex m;
    return m;
}

class LogLine {
    std::ostringstream buf_;
    bool err_;
public:
    LogLine(const char* tag, bool err) : err_(err) {
        buf_ << "[" << tag << "] ";
    }
    ~LogLine() {
        std::lock_guard<std::mutex> g(cout_mutex());
        if (err_) std::cerr << buf_.str() << std::endl;
        else      std::cout << buf_.str() << std::endl;
    }
    template<typename T>
    LogLine& operator<<(const T& v) { buf_ << v; return *this; }
};
}

#define LOG(tag)  ::logging_detail::LogLine((tag), false)
#define LOGE(tag) ::logging_detail::LogLine((tag), true)
