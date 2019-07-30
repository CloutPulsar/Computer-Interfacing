#!/usr/bin/env python

import socket
import RPi.GPIO as GPIO
import time

#LED's set ups for pins
ledNeg = 17
ledPos = 27
ledSignNeg = 23
ledSignPos = 24

GPIO.setmode(GPIO.BCM)

#GPIO SetUP for Nums
GPIO.setup(ledNeg, GPIO.OUT)
GPIO.output(ledNeg, GPIO.LOW)
GPIO.setup(ledPos, GPIO.OUT)
GPIO.output(ledPos, GPIO.LOW)

#GPIO SetUp for Sign
GPIO.setup(ledSignNeg, GPIO.OUT)
GPIO.output(ledSignNeg, GPIO.LOW)
GPIO.setup(ledSignPos, GPIO.OUT)
GPIO.output(ledSignPos, GPIO.LOW)

#Server SetUp
TCP_IP = ''
TCP_PORT =4356 
BUFFER_SIZE = 20

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)

conn, addr=s.accept()

print'Connection address:',addr
while 1:
   exp = conn.recv(BUFFER_SIZE)
   if not exp : break
   print"Client received data:",exp
   global expr
   expr = exp
   conn.send(exp)#echo
conn.close()


expr = expr.replace(" ", "")

def calc():
  if expr.find('+')  == -1 and expr.find('-') == 0:
    pos = expr.find('-',expr.find('-')+1)
  elif expr.find('-')> 0:
    pos = expr.find('-')
  else:
    pos = expr.find('+')
  num1 = int(expr[:(pos)])
  sign = expr[pos]
  num2 = int (expr[pos:])
  if num1 < 0:
    i = 0
    while i<abs(num1):
      GPIO.output(ledNeg, GPIO.HIGH)
      time.sleep(.5)
      GPIO.output(ledNeg, GPIO.LOW)
      time.sleep(.5)
      i = i+1
  else:
    i = 0
    while i<num1:
      GPIO.output(ledPos, GPIO.HIGH)
      time.sleep(.5)
      GPIO.output(ledPos, GPIO.LOW)
      time.sleep(.5)
      i = i+1
      
  if sign == '+':
    GPIO.output(ledSignPos, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(ledSignPos,GPIO.LOW)
    time.sleep(1)
  else:
    GPIO.output(ledSignNeg, GPIO.HIGH)
    time.sleep(1)
    GPIO.output(ledSignNeg, GPIO.LOW)
    time.sleep(1)
    
  if num2 < 0:
    i = 0
    while i<abs(num2):
      GPIO.output(ledNeg, GPIO.HIGH)
      time.sleep(.5)
      GPIO.output(ledNeg,GPIO.LOW)
      time.sleep(.5)
      i = i+1
  else:
    i = 0
    while i<num2:
      GPIO.output(ledPos, GPIO.HIGH)
      time.sleep(.5)
      GPIO.output(ledPos, GPIO.LOW)
      time.sleep(.5)
      i = i+1

  #Sum or Subtract the values
  if sign == '+':
    res = num1+num2
    i = 0
    while i < res:
      GPIO.output(ledPos, GPIO.HIGH)
      GPIO.output(ledSignPos, GPIO.HIGH)
      time.sleep(.5)
      GPIO.output(ledPos, GPIO.LOW)
      GPIO.output(ledSignPos, GPIO.LOW)
      time.sleep(.5)
      i = i+1
  else:
    res = num1+num2
    i = 0
    while i < abs(res):
      GPIO.output(ledNeg, GPIO.HIGH)
      GPIO.output(ledSignNeg, GPIO.HIGH)
      time.sleep(.5)
      GPIO.output(ledSignNeg, GPIO.LOW)
      GPIO.output(ledNeg, GPIO.LOW)
      time.sleep(.5)
      i = i+1
  print(res)
try:
	calc()
except KeyboardInterrupt: 
	GPIO.cleanup()  


