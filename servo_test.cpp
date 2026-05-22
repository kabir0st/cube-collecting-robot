#include <cstdio>
#include <termios.h>
#include <unistd.h>
#include "servo.h"

static void set_raw(struct termios &saved) {
    tcgetattr(STDIN_FILENO, &saved);
    struct termios raw = saved;
    raw.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

// Drive the non-blocking slew to completion. The real app calls
// servo_tick() from the 40 ms control loop; the test has no loop, so
// we spin here until the latched target is reached.
static void drain_slew() {
    while (!servo_done(0)) {
        servo_tick();
        usleep(5 * 1000); // 5 ms — well under STEP_MS, tick paces itself
    }
}

int main() {
    int angle = 90; // safe mid-range start; first call commits directly
    Servo(0, angle);
    drain_slew();
    std::printf("Servo test — a:-5  d:+5  q:quit\nangle=%d\n", angle);
    std::fflush(stdout);

    struct termios saved;
    set_raw(saved);

    char c;
    while (read(STDIN_FILENO, &c, 1) == 1 && c != 'q') {
        if (c == 'd' && angle <= 175) angle += 5;
        else if (c == 'a' && angle >= 5) angle -= 5;
        else continue;
        Servo(0, angle);
        drain_slew();
        std::printf("angle=%d\n", angle);
        std::fflush(stdout);
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &saved);
    return 0;
}
