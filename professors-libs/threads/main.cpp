/*********  compile and link with command *********
g++ mythreads.cpp main.cpp -lpthread -o threads
or
make all
**********  run with  *****************************
./threads
********** break the program with *****************
control-c
***************************************************/

#include <iostream>
#include <errno.h>
//#include <wiringPiSPI.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "mythreads.h"

using namespace std;


int main(int argc, char **argv){
	
	int Local_Counter = 0;
	pthread_t thread1, thread2;

	Global_Counter = 3;

    pthread_create(&thread1, NULL, Server, NULL);
    pthread_create(&thread2, NULL, Client, NULL);

   while(1){
		Local_Counter++;   
		if(Local_Counter%100==0){
			Send_Flag = 1;
		}
		usleep(10000);
   }
   exit(0);
}
