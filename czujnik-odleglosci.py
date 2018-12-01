import RPi.GPIO as GPIO
import time
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
 
#set GPIO Pins
GPIO_TRIGGER =  1
GPIO_ECHO = 7
GPIO_SERVO = 20

#GPIO setup
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(GPIO_SERVO, GPIO.OUT)

servo = GPIO.PWM(GPIO_SERVO, 50)
servo.start(0)

#parametry sterowania
dt = 0.01
demand_distance = 15
buffer = []

	
def distance():
	GPIO.output(GPIO_TRIGGER, True)
	time.sleep(0.00001)					#0.01ms
	GPIO.output(GPIO_TRIGGER, False)

	StartTime = time.time()
	StopTime = time.time()

	while GPIO.input(GPIO_ECHO) == 0:
		StartTime = time.time()

	while GPIO.input(GPIO_ECHO) == 1:
		StopTime = time.time()
	TimeElapsed = StopTime - StartTime
	distance = (TimeElapsed * 34300) / 2
	return distance




def meanDistance(n, buffer):
	total = 0
	for i in range(0,n):
		buffer.append(distance())
		total += buffer[i]
		#print("%d: %.3f" %(i, buffer[i]))
		time.sleep(0.02)	#50Hz
	return total/n
	
	
	
if __name__ == '__main__':
	try:
		
		while True:
			#dist = distance()
			#print ("Measured Distance = %.3f cm" % dist)
			dist = meanDistance(5,buffer)
			print("Odleglosc %.3f" %dist)

			if (dist > demand_distance + 2):
				servo.ChangeDutyCycle(1)
			elif (dist < demand_distance - 2):
				servo.ChangeDutyCycle(10)	
			else:
				servo.ChangeDutyCycle(0)
			
			buffer.clear()
			time.sleep(0.1)
			
 
        # Reset by pressing CTRL + C
	except KeyboardInterrupt:
		print("Measurement stopped by User")
		GPIO.cleanup()