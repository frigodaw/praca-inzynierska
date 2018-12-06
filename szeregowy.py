import serial
from time import *
port = serial.Serial("/dev/ttyACM16", baudrate=9600, timeout=2.0)
dane = 100
i=0

while True:
	
	if i<100:
		i+=1
	else: i=0
	rcv = port.read(12)
	print("Received data: %s" %rcv)
	port.write(("9120123").encode('ascii'))
	sleep(2)
	port.write(("9090456").encode('ascii'))
	sleep(2)
