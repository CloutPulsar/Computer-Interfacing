import RPi.GPIO as GPIO
import time
import sys
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(18,GPIO.OUT)
if (sys.argv[1] == "On"):
    GPIO.output(18, GPIO.HIGH)
elif(sys.argv[1]  =="Off"):
    GPIO.output(18,GPIO.LOW)
