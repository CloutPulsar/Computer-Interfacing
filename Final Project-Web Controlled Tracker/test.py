#!/usr/bin/env python
import serial
import time
import os
ser = serial.Serial("/dev/ttyAMA0", baudrate=50000, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=60.0)
ser.flushInput()
ser.flushOutput()
counter = 0
while True:
	line = "Herro"+ str(counter)
	ser.write(line)
	time.sleep(1)
	ser.flushOutput()
	counter+=1
