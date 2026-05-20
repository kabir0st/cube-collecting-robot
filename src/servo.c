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

static const int OPEN_DEG    = 0;
static const int CLOSED_DEG  = 120;
static const int STEP_MS     = 10;

// Last commanded angle per channel
static int s_current[2] = { 0, 0 };

static void smooth_move( int target) {
	if (target < 0)   target = 0;
	if (target > 180) target = 180;
	int ch = 0;
	int from = s_current[ch];
	if (from == target) {
		Servo(ch, target);
		return;
	}
	int step = (from < target) ? 1 : -1;
	for (int a = from; a != target; a += step) {
		Servo(ch, a);
		delay(STEP_MS);
	}
	Servo(ch, target);
	s_current[ch] = target;
}

void servo_grab()    { smooth_move(CLOSED_DEG); }
void servo_release() { smooth_move(OPEN_DEG); }

