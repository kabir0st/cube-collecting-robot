import socket
import ctypes

Header = ctypes.create_string_buffer(5)
Data = ctypes.create_string_buffer(4096)

HOST = '127.0.0.1'       # Symbolic name meaning all available interfaces
PORT = 9888              # Arbitrary non-privileged port
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(5)
conn, addr = s.accept()
file = open('testfile.txt','w')

while 1:
    number_of_ch = conn.recv_into(Header,5)
    if number_of_ch != 5: break
    if ord(Header[0]) == 165:
        Number_Of_Byte_Data = (ord(Header[2])<<16) + (ord(Header[3])<<8) + (ord(Header[4]))                                                             
        conn.recv_into(Data,Number_Of_Byte_Data)
        Quality = ord(Data[0])>>2
        Angle = ((ord(Data[1])>>1) + (ord(Data[2])<<8))>>7
        Distance = ((ord(Data[3])) + (ord(Data[4])<<8))>>2
        file.write('{} {} {} {}' .format(Quality,Angle,Distance,'\n'))      
conn.close()
file.close



