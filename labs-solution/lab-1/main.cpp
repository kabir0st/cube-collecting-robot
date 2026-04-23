#include "types.hpp"

#include <stdexcept>
#include <iostream>

int main() {
    try {
        CameraConfig config(640, 480, 120);
        ObjectColorFinder finder(config);
        finder.color_picker();
    } catch (const std::exception& e) { 
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}