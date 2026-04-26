# if you try this, you are alone!

import wiringpi as pi
import time
import struct


SPI_Channel = 1
SPI_Speed = 1000000
pi.wiringPiSPISetup(SPI_Channel,SPI_Speed)

Status = 1
Controller = 10
Current_Loop = 10
Speed_Loop = 10
Set_PWM_M0 = 10
Set_PWM_M1 = 10
Set_Speed_M0 = 10
Set_Speed_M1 = 10
Set_Digital_IO = 10
Spare = 10
Set_P_Current = 0.0
Set_P_Speed = 0.0
Set_I_Speed = 0.0
Set_Speed_Stepper = 10
Set_Position_Servo = 10
Heart_Beat = 1

Data = (Status,Controller,Current_Loop,Speed_Loop,Set_PWM_M0,Set_PWM_M1,Set_Speed_M0,Set_Speed_M1,Set_Digital_IO,Spare,Set_P_Current,Set_P_Speed,Set_I_Speed,Set_Speed_Stepper,Set_Position_Servo,Heart_Beat)
s = struct.Struct('bbhhhhhhbbfffhbb')
packed_data = s.pack(*Data)


while 1:
    RecvData = pi.wiringPiSPIDataRW(SPI_Channel,packed_data)
 #   print 'size ', s.size
 #   print 'Received data :', RecvData.count(15)
    i = 0
    for x in xrange(1, 100):
        for y in xrange(1, 10):
            i = i +1
#    print i
 #   time.sleep(1)



