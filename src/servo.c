#include <wiringPi.h>
#include "servo.h"

/*--------------------------------------- Do not change the pins --------------------------*/
const int PWM0_pin = 26; // GPIO 26 as WiringPi, <=> pin 32 (RPI ver 3B)
const int PWM1_pin = 23;  // GPIO 23 as WiringPi, <=> pin 33 (RPI ver 3B)
/*-----------------------------------------------------------------------------------------*/

static int INIT_RPI = 0;

void Init_RPI(void){

	INIT_RPI = 1;
	wiringPiSetup();
	pinMode(PWM0_pin, PWM_OUTPUT); /* set PWM0 pin as output */
	pwmSetMode(PWM_MODE_MS);
	pinMode(PWM1_pin, PWM_OUTPUT); /* set PWM1 pin as output */
	pwmSetMode(PWM_MODE_MS);
	pwmSetClock(384);  // 19.2 Mhz divided by 384.
	pwmSetRange(1000); // 19.2 Mhz divided by 384 divided by 1000 = 50Hz
}

void Servo(int Ch, int Deg) {
	if (!INIT_RPI){
		Init_RPI();
	}
	if (Ch == 0 && Deg > -1 && Deg < 181){ // range: one to two ms pwm
		pwmWrite(PWM0_pin, (50*Deg)/180 + 50);    // range 0deg = 1ms to 180deg = 2ms
	}
	else if (Ch == 1 && Deg > -1 && Deg < 181){ // range: one to two ms pwm
		pwmWrite(PWM1_pin, (50*Deg)/180 + 50);    // range 0deg = 1ms to 180deg = 2ms
	}
}

enum {
	OPEN_DEG   = 140,
	CLOSED_DEG = 75,
	// Step size + dwell between PWM updates. The PWM frame is 20 ms (50 Hz),
	// so STEP_MS must be at least one frame for the servo to actually see
	// each intermediate command. 20 ms × (65 / STEP_DEG) ≈ 140 ms total for
	// a full open↔close traversal -- fast enough not to block the control
	// loop noticeably, slow enough that peak current stays well under the
	// Pi's 5 V rail budget. A direct jump from 140° to 75° was drawing
	// enough current during the slew to brown out the rail and hang the SoC.
	STEP_DEG   = 10,
	STEP_MS    = 20
};

// Last commanded angle per channel. -1 = unknown (no previous command).
// Allows smooth-step from the previous position and makes repeated calls
// to the same target a no-op.
static int last_deg[2] = { -1, -1 };

static int clamp_deg(int d) {
	if (d < 0)   return 0;
	if (d > 180) return 180;
	return d;
}

// Walk the PWM output from the previous angle to `target_deg` in
// STEP_DEG-sized steps with STEP_MS between each step. On the very first
// call (last_deg unknown) the function snaps to target — we have no prior
// position to interpolate from, and the user is presumably holding the
// gripper before boot anyway.
static void servo_smooth_to(int ch, int target_deg) {
	if (!INIT_RPI) Init_RPI();
	if (ch != 0 && ch != 1) return;
	target_deg = clamp_deg(target_deg);

	int prev = last_deg[ch];
	if (prev < 0) {
		Servo(ch, target_deg);
		last_deg[ch] = target_deg;
		return;
	}
	if (prev == target_deg) return;

	int cur  = prev;
	int step = (target_deg > prev) ? STEP_DEG : -STEP_DEG;
	while (cur != target_deg) {
		int next = cur + step;
		if ((step > 0 && next > target_deg) ||
		    (step < 0 && next < target_deg)) {
			next = target_deg;
		}
		Servo(ch, next);
		cur = next;
		last_deg[ch] = cur;
		if (cur != target_deg) delay(STEP_MS);
	}
}

void servo_grab()    { servo_smooth_to(0, CLOSED_DEG); }
void servo_release() { servo_smooth_to(0, OPEN_DEG); }
