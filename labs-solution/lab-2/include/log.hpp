#pragma once
#include <iostream>
#include <sstream>
#include <thread>
#include <mutex>
#include <unistd.h>

// LOG("tag") << "message" prints `[pid:tid][tag] message\n` atomically.
// One global mutex serialises stdout/stderr so interleaved threads don't
// shred each other's lines.
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
        buf_ << "[" << ::getpid() << ":" << std::this_thread::get_id()
             << "][" << tag << "] ";
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
