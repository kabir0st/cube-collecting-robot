#pragma once 
#include <atomic>

namespace app {
    extern std::atomic<bool> shutdown;
    extern std::atomic<bool> shutdown_listener;
}