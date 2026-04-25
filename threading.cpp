#include <thread>
#include <chrono>
#include <iostream>
#include <mutex>
#include <atomic>

std::mutex cout_mutex;
std::atomic<bool> running{true};

void count(int thread_id) {
    {
        std::lock_guard<std::mutex> lock(cout_mutex);
        std::cout << "Thread " << thread_id << " running\n";
    }
    
    int counter = 0;
    while (running) {
        counter++;
        {
            std::lock_guard<std::mutex> lock(cout_mutex);
            std::cout << "[" << thread_id << "] : " << counter << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::seconds(thread_id));
    }
    
    {
        std::lock_guard<std::mutex> lock(cout_mutex);
        std::cout << "Thread " << thread_id << " exiting\n";
    }
}

int main() {
    std::thread t1(count, 1);
    std::thread t2(count, 2);
    
    std::this_thread::sleep_for(std::chrono::seconds(1));  // Shorter for demo
    
    {
        std::lock_guard<std::mutex> lock(cout_mutex);
        std::cout << "\nMain function - setting running to false\n";
    }
    
    running = false;
    
    t1.join();
    t2.join();
    
    std::cout << "Program exiting normally\n";
    return 0;
}