#!/usr/bin/env python
import serial
import time
import os
ser = serial.Serial("/dev/ttyAMA0", baudrate=50000, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=60.0)
ser.flushInput()
ser.flushOutput()
previous = ""
def CheckifCommands():
	filename = "info.txt"
	global previous
    #Check if the User Sent a command to move the Solar Tracker
   	if os.stat(filename).st_size != 0:
   		with open(filename) as f:
    			line =f.readline().strip('\n')
		if (previous == "Auto" and line == "Auto"):
			open('info.txt', 'w').close()
			return
		previous = line
   		ser.write(line[0])
   		open('info.txt', 'w').close()
		ser.flushOutput()
   	else:
   		return

while True:
     CheckifCommands()
     data = ser.inWaiting()
     byt = ser.read(data)
     dataSend = open('data.txt', 'w')
     dataSend.write(byt.partition(b'\0')[0])
     dataSend.close()
     time.sleep(.4)
     
     
     
     


    
