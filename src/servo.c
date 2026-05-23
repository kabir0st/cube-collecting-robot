#include <wiringPi.h>
#include "servo.h"

/*--------------------------------------- Do not change the pins --------------------------*/
const int PWM0_pin = 26; // GPIO 26 as WiringPi, <=> pin 32 (RPI ver 3B)
const int PWM1_pin = 23;  // GPIO 23 as WiringPi, <=> pin 33 (RPI ver 3B)
/*-----------------------------------------------------------------------------------------*/

enum {
	OPEN_DEG   = 35,
	CLOSED_DEG = 115,
	// PWM frame is 20 ms (50 Hz), so STEP_MS must be ≥ one frame for the
	// servo to actually see each intermediate command. A direct jump from
	// 140° to 75° draws enough current during the slew to brown out the 5 V
	// rail and hang the SoC -- step gradually instead.
	STEP_DEG   = 1,
	STEP_MS    = 20
};

static int INIT_RPI = 0;
static int cur_deg[2]    = {-1, -1}; // -1 = unknown (no command issued yet)
static int target_deg[2] = {-1, -1};
static unsigned int last_step_ms[2] = {0, 0};

void Init_RPI(void){
	INIT_RPI = 1;
	wiringPiSetup();
	pinMode(PWM0_pin, PWM_OUTPUT);
	pwmSetMode(PWM_MODE_MS);
	pinMode(PWM1_pin, PWM_OUTPUT);
	pwmSetMode(PWM_MODE_MS);
	pwmSetClock(384);  // 19.2 Mhz divided by 384.
	pwmSetRange(1000); // 19.2 Mhz divided by 384 divided by 1000 = 50Hz
}

static void pwm_write_deg(int ch, int deg) {
	int pin = (ch == 0) ? PWM0_pin : PWM1_pin;
	pwmWrite(pin, (50 * deg) / 180 + 50); // 0deg=1ms, 180deg=2ms
	cur_deg[ch] = deg;
	last_step_ms[ch] = millis();
}

void Servo(int Ch, int Deg) {
	if (!INIT_RPI) Init_RPI();
	if (Ch < 0 || Ch > 1) return;
	if (Deg < 0 || Deg > 180) return;

	target_deg[Ch] = Deg;

	// First command on this channel: physical position is unknown, so we
	// must commit it directly. From here on, servo_tick() steps gradually.
	if (cur_deg[Ch] < 0) pwm_write_deg(Ch, Deg);
}

void servo_tick(void) {
	if (!INIT_RPI) return; // nothing latched yet
	unsigned int now = millis();
	for (int ch = 0; ch < 2; ++ch) {
		if (target_deg[ch] < 0 || cur_deg[ch] < 0) continue;
		if (cur_deg[ch] == target_deg[ch]) continue;
		if (now - last_step_ms[ch] < STEP_MS) continue;

		int diff = target_deg[ch] - cur_deg[ch];
		int step = (diff >  STEP_DEG) ?  STEP_DEG
		         : (diff < -STEP_DEG) ? -STEP_DEG
		         : diff;
		pwm_write_deg(ch, cur_deg[ch] + step);
	}
}

int servo_done(int ch) {
	if (ch < 0 || ch > 1) return 1;
	if (target_deg[ch] < 0) return 1;
	return cur_deg[ch] == target_deg[ch];
}

void servo_grab()    { Servo(0, CLOSED_DEG); }
void servo_release() { Servo(0, OPEN_DEG); }
