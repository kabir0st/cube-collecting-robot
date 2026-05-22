#include <wiringPi.h>
#include "servo.h"

/*--------------------------------------- Do not change the pins --------------------------*/
const int PWM0_pin = 26; // GPIO 26 as WiringPi, <=> pin 32 (RPI ver 3B)
const int PWM1_pin = 23; // GPIO 23 as WiringPi, <=> pin 33 (RPI ver 3B)
/*-----------------------------------------------------------------------------------------*/

static int INIT_RPI = 0;

void Init_RPI(void)
{

	INIT_RPI = 1;
	wiringPiSetup();
	pinMode(PWM0_pin, PWM_OUTPUT); /* set PWM0 pin as output */
	pwmSetMode(PWM_MODE_MS);
	pinMode(PWM1_pin, PWM_OUTPUT); /* set PWM1 pin as output */
	pwmSetMode(PWM_MODE_MS);
	pwmSetClock(384);  // 19.2 Mhz divided by 384.
	pwmSetRange(1000); // 19.2 Mhz divided by 384 divided by 1000 = 50Hz
}

void Servo(int Ch, int Deg)
{
	if (!INIT_RPI)
	{
		Init_RPI();
	}
	if (Ch == 0 && Deg > -1 && Deg < 181)
	{											   // range: one to two ms pwm
		pwmWrite(PWM0_pin, (50 * Deg) / 180 + 50); // range 0deg = 1ms to 180deg = 2ms
	}
	else if (Ch == 1 && Deg > -1 && Deg < 181)
	{											   // range: one to two ms pwm
		pwmWrite(PWM1_pin, (50 * Deg) / 180 + 50); // range 0deg = 1ms to 180deg = 2ms
	}
}

enum
{
	OPEN_DEG = 140,
	CLOSED_DEG = 75,
	// Step size + dwell between PWM updates. The PWM frame is 20 ms (50 Hz),
	// so STEP_MS must be at least one frame for the servo to actually see
	// each intermediate command. 20 ms × (65 / STEP_DEG) ≈ 140 ms total for
	// a full open↔close traversal -- fast enough not to block the control
	// loop noticeably, slow enough that peak current stays well under the
	// Pi's 5 V rail budget. A direct jump from 140° to 75° was drawing
	// enough current during the slew to brown out the rail and hang the SoC.
	STEP_DEG = 10,
	STEP_MS = 20
};

// track current angle per channel so repeated calls are no-ops
static int current_deg[2] = {OPEN_DEG, OPEN_DEG};

void servo_grab()
{
	int target = CLOSED_DEG;
	while (current_deg[0] != target)
	{
		if (current_deg[0] > target)
			current_deg[0] = (current_deg[0] - STEP_DEG < target)
								 ? target
								 : current_deg[0] - STEP_DEG;
		else
			current_deg[0] = (current_deg[0] + STEP_DEG > target)
								 ? target
								 : current_deg[0] + STEP_DEG;
		Servo(0, current_deg[0]);
		delay(STEP_MS); // wiringPi delay — lets PWM frame settle
	}
}

void servo_release()
{
	int target = OPEN_DEG;
	while (current_deg[0] != target)
	{
		if (current_deg[0] < target)
			current_deg[0] = (current_deg[0] + STEP_DEG > target)
								 ? target
								 : current_deg[0] + STEP_DEG;
		else
			current_deg[0] = (current_deg[0] - STEP_DEG < target)
								 ? target
								 : current_deg[0] - STEP_DEG;
		Servo(0, current_deg[0]);
		delay(STEP_MS); // wiringPi delay — lets PWM frame settle
	}
}

// void servo_grab() { Servo(0, CLOSED_DEG); }
// void servo_release() { Servo(0, OPEN_DEG); }
