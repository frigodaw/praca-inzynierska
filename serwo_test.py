import sys
import RPi.GPIO as GPIO
from time import *

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO_SERVO1 = 16
GPIO_SERVO2 = 20
GPIO_SERVO3 = 21
x = 0

GPIO.setup(GPIO_SERVO1, GPIO.OUT)								#serwo1: podstawa
GPIO.setup(GPIO_SERVO2, GPIO.OUT)								#serwo2: ramie
GPIO.setup(GPIO_SERVO3, GPIO.OUT)								#serwo3: 360 teleskop

serwo1 = GPIO.PWM(GPIO_SERVO1, 50)
serwo2 = GPIO.PWM(GPIO_SERVO2, 50)
serwo3 = GPIO.PWM(GPIO_SERVO3, 50)

#serwo1.start(x)		
serwo2.start(x)
#serwo3.start(x)

while True:
	if x < 10:
		x += 0.5
		#serwo1.ChangeDutyCycle(x)
		serwo2.ChangeDutyCycle(x)
	else: 
		x = 0
		#serwo1.ChangeDutyCycle(x)
		serwo2.ChangeDutyCycle(x)
	print("Wypelnienie serwo  %0.3f" %x)
	sleep(1)