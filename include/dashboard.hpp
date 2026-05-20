#pragma once
#include <atomic>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>
#include <unistd.h>

#include <loggers.hpp>

// escape sequences for std::cout.

// \033[H — move cursor to top-left
// \033[2J — clear screen
// \033[<N>A — move cursor up N lines
// \033[K — erase from cursor to end of line
// \033[s / \033[u — save / restore cursor position
// \033[?25l / \033[?25h — hide / show cursor (reduces flicker)
// \033[?1049h / \033[?1049l — enter / leave alternate screen buffer (vim/less-style)


class Dashboard {
    static constexpr int COL = 12;

    struct Row {
        std::string name;
        std::vector<std::pair<std::string, std::string>> cells;
    };

    std::mutex dash_data_mutex;
    std::vector<Row> dash_rows;
    std::atomic<bool> dash_started{false};

public:
    ~Dashboard() { stop(); }
    void start(int dashboard_rows = 10) {
        if (dash_started.exchange(true)) return;
        std::lock_guard<std::mutex> g(logging_detail::cout_mutex());
        std::cout << "\033[?1049h\033[?25l\033[2J\033[H"
                  << "\033[" << (dashboard_rows + 1) << ";999r"
                  << "\033[" << (dashboard_rows + 1) << ";1H"
                  << std::flush;
    }

    void stop() {
        if (!dash_started.exchange(false)) return;
    }

    void add_row(const std::string& name) {
        std::lock_guard<std::mutex> g(dash_data_mutex);
        for (auto& r : dash_rows) {
            if (r.name == name) return;
        }
        dash_rows.push_back(Row{name, {}});
    }

    template <typename T>
    static typename std::enable_if<std::is_floating_point<T>::value>::type
    format_value(std::ostringstream& os, const T& value) {
        os << std::fixed << std::setprecision(2) << value;
    }

    template <typename T>
    static typename std::enable_if<!std::is_floating_point<T>::value>::type
    format_value(std::ostringstream& os, const T& value) {
        os << value;
    }

    template <typename T>
    void set(const std::string& row, const std::string& key, const T& value) {
        std::ostringstream os;
        format_value(os, value);
        std::string formatted = os.str();

        std::lock_guard<std::mutex> g(dash_data_mutex);
        for (auto& r : dash_rows) {
            if (r.name != row) continue;
            for (auto& cell : r.cells) {
                if (cell.first == key) {
                    cell.second = std::move(formatted);
                    return;
                }
            }
            r.cells.emplace_back(key, std::move(formatted));
            return;
        }
    }

    void render() {
        if (!dash_started) return;
        std::ostringstream frame;
        frame << "\033[s\033[H";
        {
            std::lock_guard<std::mutex> lock(dash_data_mutex);
            for (auto& row : dash_rows) {
                frame << std::left << row.name << "\033[K\n";

                for (auto& cell : row.cells) {
                    frame << std::left << std::setw(COL) << cell.first;
                }
                frame << "\033[K\n";

                for (auto& cell : row.cells) {
                    frame << std::left << std::setw(COL) << cell.second;
                }
                frame << "\033[K\n";

                frame << "----------------------------\033[K\n";
            }
        }
        frame << "\033[u";
        std::lock_guard<std::mutex> g(logging_detail::cout_mutex());
        std::cout << frame.str() << std::flush;
    }
};
