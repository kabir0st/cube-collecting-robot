#include "types.hpp"
#include "calibration.hpp"
#include <stdexcept>
#include <iostream>

int main() {
    try {
        CameraConfig config(640, 480, 120);
        CalibrateCamera calibration(config);
        calibration.start_calibration();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}