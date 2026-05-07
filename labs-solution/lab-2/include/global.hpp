#pragma once
#include <atomic>

// Single global kill switch. Set by SIGINT or by any thread that decides
// the program should wind down. Every blocking loop in the project
// polls this and exits cleanly when it becomes true.
namespace app {
    extern std::atomic<bool> shutdown;
}
