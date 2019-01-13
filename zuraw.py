import math
import numpy as np			
import RPi.GPIO as GPIO
import serial
import sys
import threading
from time import *


def degToRad(deg):
	return deg*math.pi/180

def radToDeg(rad):
	return rad*180/math.pi
	
def channelSetup(channel_XYZ, channel_JOINT):		
	if GPIO.input(channel_XYZ) == GPIO.LOW:
		print("Tryb: XYZ")
		return 'XYZ'
	elif GPIO.input(channel_JOINT) == GPIO.LOW:
		print("Tryb: JOINT")
		return 'JOINT'
	else:
		print("Domyslny tryb pracy: XYZ")
		return 'XYZ'
	
class ParametersDH:

	def __init__(self,fi,d,a,alfa):
		self.fi = degToRad(fi)
		self.d = d
		self.a = a
		self.alfa = degToRad(alfa)
		
		self.rotZ()
		self.tranZ()
		self.tranX()
		self.rotX()
		self.A = (((self.rZ).dot(self.tZ)).dot(self.tX)).dot(self.rX)
		
	def rotZ(self):
		self.rZ = np.array([[math.cos(self.fi),-math.sin(self.fi),0,0],[math.sin(self.fi), math.cos(self.fi),0,0],[0,0,1,0],[0,0,0,1]])

	def tranZ(self):
		self.tZ = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,self.d],[0,0,0,1]])

	def tranX(self):
		self.tX = np.array([[1,0,0,self.a],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

	def rotX(self):
		self.rX = np.array([[1,0,0,0], [0,math.cos(self.alfa),-math.sin(self.alfa),0], [0,math.sin(self.alfa),math.cos(self.alfa),0], [0,0,0,1]])

		
class Kinematics:
	
	def __init__(self, fi1, fi2, fi3, l1, l2, d3, l4, operating_mode):
		self.fi1 = fi1
		self.fi2 = fi2
		self.fi3 = fi3
		self.l1 = l1
		self.l2 = l2
		self.d3 = d3
		self.l4 = l4
		self.operating_mode = operating_mode
		
		self.forwardKinematics(self.fi1, self.fi3, self.d3)
		self.inverseKinematics(self.X, self.Y, self.Z)
		sleep(2)
		threadSerial.sendData(self.fi1, self.fi3, self.d3)
		
	def forwardKinematics(self, fi1, fi3, d3):
		try:
			self.X_mem = self.X		#kopia zapasowa wspolrzednych XYZ
			self.Y_mem = self.Y
			self.Z_mem = self.Z
		except: AttributeError
		
		czlon1 = ParametersDH(fi1, self.l1, 0, 90)
		czlon2 = ParametersDH(self.fi2, 0, self.l2, 0)
		czlon3 = ParametersDH(-(self.fi2-fi3), 0, d3+self.l4, 0)
		czlon4 = ParametersDH(-90, 0, 0, -90)
						
		self.A = (((czlon1.A).dot(czlon2.A)).dot(czlon3.A)).dot(czlon4.A)
		self.X = self.A[0][3]
		self.Y = self.A[1][3]
		self.Z = self.A[2][3]

		print("X: %.3f" %self.X)
		print("Y: %.3f" %self.Y)
		print("Z: %.3f" %self.Z)
		
	def inverseKinematics(self,X,Y,Z):
		self.X = X
		self.Y = Y
		self.Z = Z
		
		self.fi1_mem = self.fi1		#kopia zapasowa wspolrzednych JOINT
		self.fi3_mem = self.fi3
		self.d3_mem = self.d3

		self.fi1 = math.atan(self.Y/ self.X)
		a = self.l2*math.cos(degToRad(self.fi2))
		b = self.Z-self.l1-self.l2*math.sin(degToRad(self.fi2))
		self.fi3 = math.pi/2-math.atan(self.X/(b*math.cos(self.fi1))-a/b)
		self.d3 = b/math.sin(self.fi3)-self.l4

		self.fi1 = radToDeg(self.fi1)
		self.fi3 = radToDeg(self.fi3)

		print("fi1: %.3f" %self.fi1)
		print("fi3: %.3f" %self.fi3)
		print("d3: %.3f" %self.d3)

	def checkRange(self, angle_1, angle_3, length_3, angle_1_min, angle_1_max, angle_3_min, angle_3_max, length_3_min, length_3_max):
		if (angle_1 < angle_1_min) or (angle_1 > angle_1_max) or (angle_3 < angle_3_min) or (angle_3 > angle_3_max) or (length_3 < length_3_min) or (length_3 > length_3_max):
			return -1		#przekroczenie wartosci
		else: return 1		#wartosc poprawna
				
	def controlXYZ(self):
		print("X: %.3f" %self.X )
		print("Y: %.3f" %self.Y )
		print("Z: %.3f" %self.Z )
		self.inverseKinematics(self.X, self.Y, self.Z)
				
		if self.checkRange(self.fi1, self.fi3, self.d3, fi1_min, fi1_max, fi3_min, fi3_max, d3_min, d3_max) == -1:	#przekroczono wartosci katow
			dioda_alarm.ChangeDutyCycle(50)
			print("Przekroczono wartosci czlonow")
			print("Nie wyslano sygnalow sterujacych")
			self.fi1 = self.fi1_mem		 #przywrocenie wspolrzednych z pamieci
			self.fi3 = self.fi3_mem
			self.d3 = self.d3_mem
			self.forwardKinematics(self.fi1, self.fi3, self.d3)
			self.inverseKinematics(self.X, self.Y, self.Z)
		
		else: 
			dioda_alarm.ChangeDutyCycle(0)
			threadSerial.sendData(self.fi1, self.fi3, self.d3)
		
	def controlJOINT(self):
		print("fi1: %.3f" %self.fi1)
		print("fi3: %.3f" %self.fi3)
		print("d3: %.3f" %self.d3)
		self.forwardKinematics(self.fi1, self.fi3, self.d3)
		
		if self.checkRange(self.fi1, self.fi3, self.d3, fi1_min, fi1_max, fi3_min, fi3_max, d3_min, d3_max) == -1:	#przekroczono wartosci katow
			dioda_alarm.ChangeDutyCycle(50)
			print("Przekroczono wartosci czlonow")
			print("Nie wyslano sygnalow sterujacych")
			self.X = self.X_mem		#przywrocenie wspolrzednych z pamieci
			self.Y = self.Y_mem
			self.Z = self.Z_mem
			self.inverseKinematics(self.X, self.Y, self.Z)
			self.forwardKinematics(self.fi1, self.fi3, self.d3)
			
		else: 
			dioda_alarm.ChangeDutyCycle(0)
			threadSerial.sendData(self.fi1, self.fi3, self.d3)

class NewThread(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		self.connectionLost = 0

	def run(self):	
		while True:
			try:
				self.port = serial.Serial("/dev/arduino", baudrate=9600, timeout=3.0)
				self.isConnected = 1
				GPIO.output(GPIO_LED_G,	GPIO.HIGH)
			except:
				print("Nie mozna polaczyc z Arduino!")
				self.isConnected = 0
				self.connectionLost = 1
				GPIO.output(GPIO_LED_G,	GPIO.LOW)
			
			if ((self.isConnected == 1) and (self.connectionLost == 1)):
				sleep(3)
				print("Przywrocono komunikacje szeregowa z Arduino!")
				self.sendData(zuraw.fi1, zuraw.fi3, zuraw.d3)
				self.connectionLost = 0
			sleep(0.5)

	def sendData(self, fi1, fi3, d3):
		if (self.isConnected == 1):
			buffer = '\n'
			fi1 = str(int(round(fi1*1000)))
			fi3 = str(int(round(fi3*1000)))
			d3 = str(int(round(d3*1000)))
			buffer = fi1+'x'+fi3+'x'+d3+'\n';
			self.port.write((buffer).encode('ascii'))
			print("Wyslano: %s " %buffer)
				
				
#GPIO PINOUT BCM
GPIO_X_UP = 4
GPIO_X_DOWN = 17
GPIO_Y_UP = 27
GPIO_Y_DOWN = 22
GPIO_Z_UP = 10
GPIO_Z_DOWN = 9
GPIO_XYZ = 11
GPIO_JOINT = 0
GPIO_LED_G = 19
GPIO_LED_Y = 26

#KONFIGURACJA WEJSC I WYJSC
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(GPIO_X_UP, GPIO.IN, pull_up_down=GPIO.PUD_UP)		#kierunek X up
GPIO.setup(GPIO_X_DOWN, GPIO.IN, pull_up_down=GPIO.PUD_UP)		#kierunek X down
GPIO.setup(GPIO_Y_UP, GPIO.IN, pull_up_down=GPIO.PUD_UP)		#kierunek Y up
GPIO.setup(GPIO_Y_DOWN, GPIO.IN, pull_up_down=GPIO.PUD_UP)		#kierunek Y down
GPIO.setup(GPIO_Z_UP, GPIO.IN, pull_up_down=GPIO.PUD_UP)		#kierunek Z up
GPIO.setup(GPIO_Z_DOWN, GPIO.IN, pull_up_down=GPIO.PUD_UP)		#kierunek Z down
GPIO.setup(GPIO_XYZ, GPIO.IN, pull_up_down=GPIO.PUD_UP)			#sterowanie XYZ
GPIO.setup(GPIO_JOINT, GPIO.IN, pull_up_down=GPIO.PUD_UP)		#sterowanie JOINT
GPIO.setup(GPIO_LED_G, GPIO.OUT)								#LED komunikacja szeregowa
GPIO.setup(GPIO_LED_Y, GPIO.OUT)								#LED pozycja skrajna
dioda_alarm = GPIO.PWM(GPIO_LED_Y, 1)
dioda_alarm.start(0)

#PARAMETRY STARTOWE VAR
fi1 = 0		#deg
fi3 = 30	#deg
d3 = 100 	#mm

#POZOSTALE WYMIARY CZLONOW
fi2 = 150	#deg
l1 = 0		#mm
l2 = 135	#mm
l4 = 175	#mm

#WARUNKI KRANCOWE
fi1_min = -85	#deg
fi1_max = 85	#deg
fi3_min = 10	#deg
fi3_max = 75	#deg
d3_min = 60		#mm
d3_max = 165	#mm
	
#PARAMETRY NARASTANIA WSPOLRZEDNYCH
dx = 3		#mm
dy = 3		#mm
dz = 3 		#mm
dfi1 = 3	#deg
dfi3 = 3	#deg
dd3 = 3		#mm
dt = 0.1	#s
btime = 200 #bouncetime ms

#OBSLUGA PRZERWAN OD PRZYCISKOW
def callback_upX_upF1(channel):
	while (GPIO.input(channel) == GPIO.LOW) and (threadSerial.isConnected == 1):
		if zuraw.operating_mode == 'XYZ':
			print("\nRuch X+")
			zuraw.X += dx
			zuraw.controlXYZ()
			sleep(dt)
		
		elif zuraw.operating_mode == 'JOINT':
			print("\nRuch fi1+")
			zuraw.fi1 += dfi1
			zuraw.controlJOINT()
			sleep(dt)

def callback_downX_downF1(channel):
	while (GPIO.input(channel) == GPIO.LOW) and (threadSerial.isConnected == 1):
		if zuraw.operating_mode == 'XYZ':
			print("\nRuch X-")
			zuraw.X -= dx
			zuraw.controlXYZ()
			sleep(dt)
		
		elif zuraw.operating_mode == 'JOINT':
			print("\nRuch fi1-")
			zuraw.fi1 -= dfi1
			zuraw.controlJOINT()
			sleep(dt)
		
	
def callback_upY_upF3(channel):
	while (GPIO.input(channel) == GPIO.LOW) and (threadSerial.isConnected == 1):
		if zuraw.operating_mode == 'XYZ':
			print("\nRuch Y+")
			zuraw.Y += dy
			zuraw.controlXYZ()
			sleep(dt)
		
		elif zuraw.operating_mode == 'JOINT':
			print("\nRuch fi3+")
			zuraw.fi3 += dfi3
			zuraw.controlJOINT()
			sleep(dt)

def callback_downY_downF3(channel):
	while (GPIO.input(channel) == GPIO.LOW) and (threadSerial.isConnected == 1):
		if zuraw.operating_mode == 'XYZ':
			print("\nRuch Y-")
			zuraw.Y -= dy
			zuraw.controlXYZ()
			sleep(dt)
		
		elif zuraw.operating_mode == 'JOINT':
			print("\nRuch fi3-")
			zuraw.fi3 -= dfi3
			zuraw.controlJOINT()
			sleep(dt)
	
def callback_upZ_upD3(channel):
	while (GPIO.input(channel) == GPIO.LOW) and (threadSerial.isConnected == 1):
		if zuraw.operating_mode == 'XYZ':
			print("\nRuch Z+")
			zuraw.Z += dz
			zuraw.controlXYZ()
			sleep(dt)
		
		elif zuraw.operating_mode == 'JOINT':
			print("\nRuch d3+")
			zuraw.d3 += dd3
			zuraw.controlJOINT()
			sleep(dt)

def callback_downZ_downD3(channel):
	while (GPIO.input(channel) == GPIO.LOW) and (threadSerial.isConnected == 1):
		if zuraw.operating_mode == 'XYZ':
			print("\nRuch Z-")
			zuraw.Z -= dz
			zuraw.controlXYZ()
			sleep(dt)
		
		elif zuraw.operating_mode == 'JOINT':
			print("\nRuch d3-")
			zuraw.d3 -= dd3
			zuraw.controlJOINT()
			sleep(dt)	

def callbackModeXYZ(channel):
	if (GPIO.input(channel) == GPIO.LOW) and (zuraw.operating_mode == 'JOINT'):
		zuraw.operating_mode = 'XYZ'	
		print("Tryb pracy: XYZ")	

def callbackModeJOINT(channel):
	if (GPIO.input(channel) == GPIO.LOW) and (zuraw.operating_mode == 'XYZ'):
		zuraw.operating_mode = 'JOINT'
		print("Tryb pracy: JOINT")

#KONFIGURACJA PRZERWAN
GPIO.add_event_detect(GPIO_X_UP, GPIO.FALLING, callback=callback_upX_upF1, bouncetime=btime)	
GPIO.add_event_detect(GPIO_X_DOWN, GPIO.FALLING, callback=callback_downX_downF1, bouncetime=btime)
GPIO.add_event_detect(GPIO_Y_UP, GPIO.FALLING, callback=callback_upY_upF3, bouncetime=btime)
GPIO.add_event_detect(GPIO_Y_DOWN, GPIO.FALLING, callback=callback_downY_downF3, bouncetime=btime)	
GPIO.add_event_detect(GPIO_Z_UP, GPIO.FALLING, callback=callback_upZ_upD3, bouncetime=btime)
GPIO.add_event_detect(GPIO_Z_DOWN, GPIO.FALLING, callback=callback_downZ_downD3, bouncetime=btime)
GPIO.add_event_detect(GPIO_XYZ, GPIO.FALLING, callback=callbackModeXYZ, bouncetime=btime)
GPIO.add_event_detect(GPIO_JOINT, GPIO.FALLING, callback=callbackModeJOINT, bouncetime=btime)

	
#UTWORZENIE OBIEKTOW
threadSerial = NewThread()
threadSerial.start()
print("\nPozycja startowa. Na podstawie wymiarow geometrycznych obliczane jest polozenie efektora (kinematyka prosta).\n")
print("START:")
zuraw = Kinematics(fi1,fi2,fi3,l1,l2,d3,l4, channelSetup(11,0))
print("Nacisnij przycisk aby poruszac manipulatorem.")	


	
#ZAPETLENIE PROGRAMU
while True:
	try: 		
		sleep(0.5)
	except: KeyboardInterrupt

GPIO.cleanup()

