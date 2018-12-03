import RPi.GPIO as GPIO
import time
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
 
#set GPIO Pins
GPIO_TRIGGER =  1
GPIO_ECHO = 7
GPIO_SERVO = 21

#GPIO setup
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(GPIO_SERVO, GPIO.OUT)

serwo3 = GPIO.PWM(GPIO_SERVO, 50)
serwo3.start(0)

#parametry sterowania
dt = 0.01

	
def distance(demand_distance):			#dlugosc do przejechania
	n = 10								#ilosc elementow do liczenia sredniej
	total = 0
	d3_position = 0
	buffer = []
	
	while (d3_position > demand_distance + 2) or (d3_position < demand_distance - 2):
		
		for i in range(0, n):
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
			distance = (TimeElapsed * 343000) / 2	#[mm]
			buffer.append(distance)
			total += buffer[i]
			time.sleep(0.02)	#50Hz
		d3_position = total/n
		buffer.clear()
		total = 0
		
		print("Odleglosc %.3f" %d3_position)
		if (d3_position > demand_distance + 2):
			serwo3.ChangeDutyCycle(1)
		elif (d3_position < demand_distance - 2):
			serwo3.ChangeDutyCycle(10)	
		else:
			serwo3.ChangeDutyCycle(0)
		

if __name__ == '__main__':
	try:
		
		while True:
			
			distance(100)
			
 
        # Reset by pressing CTRL + C
	except KeyboardInterrupt:
		print("Measurement stopped by User")
		GPIO.cleanup()
