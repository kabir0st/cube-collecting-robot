import socket

TCP_IP = '127.0.0.1'
TCP_PORT = 9887
BUFFER_SIZE = 1024

s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)

s.connect((TCP_IP,TCP_PORT))
s.sendall(bytes('\x20\x00', 'UTF-8'))
s.close()

