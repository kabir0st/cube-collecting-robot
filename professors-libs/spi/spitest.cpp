/*********  compile and link with command *********
g++ spitest.cpp -lwiringPi -o spitest
**********  run with  *****************************
./spitest
********** break the program with *****************
control-c
***************************************************/

#include <iostream>
#include <errno.h>
#include <wiringPiSPI.h>
#include <unistd.h>
#include "spi_com.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

using namespace std;
static const int CHANNEL = 1;

int main(void)
{
   int fd, result;
	int Counter = 0;
   unsigned char buffer[100];
   TYPE_White_Board_RX *pWhite_Board_RX;
   TYPE_White_Board_TX *pWhite_Board_TX;
	FILE *fid, *pos_fid , *pos_fid2;
	int Done = 0;
	int toggle = 1;
	int Home  = 0;
	int Dummy;
	int Des_Pos;
	int Des_Pos2;
        int enc_offset;
        int enc_offset2;

   cout << "Initializing" << endl ;

   pWhite_Board_TX = (TYPE_White_Board_TX *)malloc(sizeof(TYPE_White_Board_TX));
	if(!pWhite_Board_TX){
		   cout << "Couldn't allocate" << endl ;
			exit;
	}
	pWhite_Board_RX = (TYPE_White_Board_RX *)buffer;
	
   pWhite_Board_TX->Status = 0;
   pWhite_Board_TX->Control = 0;
   pWhite_Board_TX->Current_Loop = 0;
   pWhite_Board_TX->Speed_Loop = 0;
   pWhite_Board_TX->Set_PWM_M0 = 0;
   pWhite_Board_TX->Set_PWM_M1 = 0;
   pWhite_Board_TX->Set_Speed_M0 = 0;
   pWhite_Board_TX->Set_Speed_M1 = 0;
   pWhite_Board_TX->Digital_Out = 0;
   pWhite_Board_TX->Spare1 = 0;
   pWhite_Board_TX->P_Value_Current = 0.0;
   pWhite_Board_TX->P_Value_Speed = 0.0;
   pWhite_Board_TX->I_Value_Speed = 0.0;
   pWhite_Board_TX->Speed_Stepper = 0;
   pWhite_Board_TX->Position_Servo = 0;
   pWhite_Board_TX->Heart_Beat = 0;

   fd = wiringPiSPISetup(CHANNEL, 4000000);
	
   fid = fopen("temp","w");
   pos_fid = fopen("positions.txt","r");
   pos_fid2 = fopen("positions2.txt","r");

   pWhite_Board_TX->Status = 1;
   pWhite_Board_TX->Set_Speed_M0 = 0;
   pWhite_Board_TX->Set_Speed_M1 = 0;
   pWhite_Board_TX->Position_Servo = 0;
   pWhite_Board_TX->Heart_Beat = 1;

   memcpy(buffer,pWhite_Board_TX,32);
   result = wiringPiSPIDataRW(CHANNEL, buffer, 32);
    
   enc_offset = pWhite_Board_RX->Position_M0;
   enc_offset2 = pWhite_Board_RX->Position_M1;

   /*
while(!Home){
	if(pWhite_Board_RX->Position_M0 < 5 && pWhite_Board_RX->Position_M0 > -5 && pWhite_Board_RX->Speed_M0 < 5 && pWhite_Board_RX->Speed_M0 > -5){
		Home = 1;
	}
	pWhite_Board_TX->Status = 1;
	Dummy = -pWhite_Board_RX->Position_M0;
	if(Dummy > 60)
		Dummy = 60;
	else if(Dummy < -60)
		Dummy = -60;
	pWhite_Board_TX->Set_Speed_M0 = Dummy/4;
        pWhite_Board_TX->Set_Speed_M1 = Dummy/4;
	pWhite_Board_TX->Heart_Beat = 1;

       memcpy(buffer,pWhite_Board_TX,32);
       result = wiringPiSPIDataRW(CHANNEL, buffer, 32);

//fprintf(fid,"%d %d %d %d %d %d\n",pWhite_Board_RX->Position_M0,pWhite_Board_RX->Position_M1, Des_Pos,Des_Pos2, pWhite_Board_TX->Set_Speed_M0,pWhite_Board_TX->Set_Speed_M1);

       usleep(5000); // sleep in micro sek
   }
*/
   cout << "Start Pos Control" << endl ;
pWhite_Board_TX->Set_Speed_M0 = 0;
pWhite_Board_TX->Set_Speed_M1 = 0;
while (Counter < 500) {
Counter++;
result = wiringPiSPIDataRW(CHANNEL, buffer, 32);
usleep(10000);
}
   while(fscanf(pos_fid,"%d",&Des_Pos) != EOF){
      fscanf(pos_fid2,"%d",&Des_Pos2); 
	//Counter++;
	pWhite_Board_TX->Status = 1;
	pWhite_Board_TX->Set_Speed_M0 =  0.1*(Des_Pos + enc_offset - pWhite_Board_RX->Position_M0);
	pWhite_Board_TX->Set_Speed_M1 = 0.1*(Des_Pos2 + enc_offset2 - pWhite_Board_RX->Position_M1);
	if(pWhite_Board_RX->Digital_In & 0x01 == 0x01)
		pWhite_Board_TX->Digital_Out = 0x01;
	else
		pWhite_Board_TX->Digital_Out = 0x00;
	//pWhite_Board_TX->Position_Servo = Counter & 0xff;
	pWhite_Board_TX->Heart_Beat = 1;

       memcpy(buffer,pWhite_Board_TX,32);
       result = wiringPiSPIDataRW(CHANNEL, buffer, 32);
fprintf(fid,"%d  %d %d\n",pWhite_Board_RX->Position_M0, Des_Pos,pWhite_Board_RX->Position_M1, Des_Pos2, pWhite_Board_TX->Set_Speed_M0);
       usleep(10000); // sleep in micro sek
   }
   cout << "End program" << endl ;
   fclose(fid);
   fclose(pos_fid);
   fclose(pos_fid2);
}