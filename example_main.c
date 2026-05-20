/*
*   compile and link with; c++ -Wall -o servo servo.c example_main.c -lwiringPi
*   run with: sudo ./servo
*   exit with: control/c
*/

#include <wiringPi.h>
#include "servo.h"

int main(){
	int i =0;
	while(1){
		i = (i +1)%180;
		Servo(0,i);
		delay(500);
	}
	return 0;
}