#!/usr/bin/env python 

import socket

TCP_IP = '192.168.0.45'
TCP_PORT = 4356
BUFFER_SIZE = 1024
expr = raw_input("Please Enter your expression [E.G. 3+3]: ")

  
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
s.send(expr)
data = s.recv(BUFFER_SIZE)

s.close()
print "Server received data:", data


 
