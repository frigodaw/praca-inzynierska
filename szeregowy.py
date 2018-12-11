import serial
from time import *
port = serial.Serial("/dev/arduino", baudrate=9600, timeout=2.0)
dane = 100
i=0

while True:
	
	
	buffer = "4567x7654x45678x" + '\n'
	port.write((buffer).encode('ascii'))
	print(port.readline())
	print("Poszlo: %s " %buffer)
	sleep(1)
	
	buffer = "2113x67877x86697x" + '\n'
	port.write((buffer).encode('ascii'))
	print(port.readline())
	print("Poszlo: %s " %buffer)
	sleep(1)
