import numpy as np				#biblioteka macierzy
import math
import RPi.GPIO as GPIO
import sys
from time import *

def degToRad(stopnie):
	return stopnie*math.pi/180

def radToDeg(stopnie):
	return stopnie*180/math.pi
	
def fiToPWM(stopnie):  
	return 5/180*stopnie+5

def d3ToPWM(odleglosc): #regulator
    return 10

def borderControl(kat1, kat3, dlugosc3, kat1_min, kat1_max, kat3_min, kat3_max, dlugosc3_min, dlugosc3_max):
	if (kat1 < kat1_min) or (kat1 > kat1_max) or (kat3 < kat3_min) or (kat3 > kat3_max) or (dlugosc3 < dlugosc3_min) or (dlugosc3 > dlugosc3_max):
		return -1		#przekroczenie wartosci
	else: return 1		#wartosc poprawna
	
def channelSetup(channel_XYZ, channel_JOINT):		
	if GPIO.input(channel_XYZ) == GPIO.LOW:
		print("Tryb: XYZ")
		return 'XYZ'
	elif GPIO.input(channel_JOINT) == GPIO.LOW:
		print("Tryb: JOINT")
		return 'JOINT'
	else:
		print("Domyslny tryb pracy: JOINT")
		return 'JOINT'

class NotacjaDH:

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

class Kinematyka:
	
	#WARUNKI KRANCOWE
	fi1_min = -85	#deg
	fi1_max = 85	#deg
	fi3_min = 0	#deg
	fi3_max = 70	#deg
	d3_min = 40	#mm
	d3_max = 130	#mm
  	
	def __init__(self, fi1, fi2, fi3, l1, l2, d3, l4, tryb_pracy):
		
		self.fi1 = fi1
		self.fi2 = fi2
		self.fi3 = fi3
		self.l1 = l1
		self.l2 = l2
		self.d3 = d3
		self.l4 = l4
		self.tryb_pracy = tryb_pracy
		
		self.kinematykaProsta(self.fi1, self.fi3, self.d3)
		self.kinematykaOdwrotna(self.X, self.Y, self.Z)
			
		serwo1.ChangeDutyCycle(fiToPWM(self.fi1))
		serwo2.ChangeDutyCycle(fiToPWM(self.fi3))
		serwo3.ChangeDutyCycle(d3ToPWM(self.d3))     #to musi byc regulowane
		
	def kinematykaProsta(self, fi1, fi3, d3):
		try:
			self.X_mem = self.X		#kopia zapasowa wspolrzednych XYZ
			self.Y_mem = self.Y
			self.Z_mem = self.Z
		except: AttributeError
		
		czlon1 = NotacjaDH(fi1, self.l1, 0, 90)
		czlon2 = NotacjaDH(self.fi2, 0, self.l2, 0)
		czlon3 = NotacjaDH(-(self.fi2-fi3), 0, d3+self.l4, 0)
		czlon4 = NotacjaDH(-90, 0, 0, -90)
						
		self.A = (((czlon1.A).dot(czlon2.A)).dot(czlon3.A)).dot(czlon4.A)
		self.X = self.A[0][3]
		self.Y = self.A[1][3]
		self.Z = self.A[2][3]

		print("X: %.4f" %self.X)
		print("Y: %.4f" %self.Y)
		print("Z: %.4f" %self.Z)
		
	def kinematykaOdwrotna(self,X,Y,Z):
		self.X = X
		self.Y = Y
		self.Z = Z
		
		self.fi1_mem = self.fi1		#kopia zapasowa wspolrzednych JOINT
		self.fi3_mem = self.fi3
		self.d3_mem = self.d3

		self.fi1 = math.atan(self.Y/ self.X)
		a = self.l2*math.cos(degToRad(self.fi2))
		b = self.Z-self.l1-self.l2*math.sin(degToRad(self.fi2))
		self.fi3 = math.pi/2-math.atan(self.X-a*math.cos(self.fi1)/ b*math.cos(self.fi1))
		self.d3 = b/math.sin(self.fi3)-self.l4

		self.fi1 = radToDeg(self.fi1)
		self.fi3 = radToDeg(self.fi3)

		print("fi1: %.4f" %self.fi1)
		print("fi3: %.4f" %self.fi3)
		print("d3: %.4f" %self.d3)
		
	def sterowanieXYZ(self):
		print("X: %.4f" %self.X )
		print("Y: %.4f" %self.Y )
		print("Z: %.4f" %self.Z )
		self.kinematykaOdwrotna(self.X, self.Y, self.Z)
				
		if borderControl(self.fi1, self.fi3, self.d3, self.fi1_min, self.fi1_max, self.fi3_min, self.fi3_max, self.d3_min, self.d3_max) == -1:	#przekroczono wartosci katow
			dioda_alarm.ChangeDutyCycle(50)
			print("Przekroczono wartosci czlonow")
			print("Nie wyslano sygnalow sterujacych")
			self.fi1 = self.fi1_mem      #przywrocenie wspolrzednych z pamieci
			self.fi3 = self.fi3_mem
			self.d3 = self.d3_mem
			self.kinematykaProsta(self.fi1, self.fi3, self.d3)
			self.kinematykaOdwrotna(self.X, self.Y, self.Z)
		
		else: 
			dioda_alarm.ChangeDutyCycle(0)
			print("Wyslano sygnaly sterujace")
			serwo1.ChangeDutyCycle(fiToPWM(self.fi1))
			serwo2.ChangeDutyCycle(fiToPWM(self.fi3))
			serwo3.ChangeDutyCycle(d3ToPWM(self.d3))     #to musi byc regulowane
            #while (d3_rzeczywiste > (self.d3+3)) and (d3_rzeczywiste < (self.d3-3))
                #if d3_rzeczywiste > (self.d3+3):
                    #serwo3.ChangeDutyCycle(WARTOSC_TYL)
                #elif d3_rzeczywiste < (self.d3-3):
                    #serwo3.ChangeDutyCycle(WARTOSC_PRZOD)
		
	def sterowanieJOINT(self):
		print("fi1: %.4f" %self.fi1)
		print("fi3: %.4f" %self.fi3)
		print("d3: %.4f" %self.d3)
		self.kinematykaProsta(self.fi1, self.fi3, self.d3)
		
		if borderControl(self.fi1, self.fi3, self.d3, self.fi1_min, self.fi1_max, self.fi3_min, self.fi3_max, self.d3_min, self.d3_max) == -1:	#przekroczono wartosci katow
			dioda_alarm.ChangeDutyCycle(50)
			print("Przekroczono wartosci czlonow")
			print("Nie wyslano sygnalow sterujacych")
			self.X = self.X_mem          #przywrocenie wspolrzednych z pamieci
			self.Y = self.Y_mem
			self.Z = self.Z_mem
			print("XYZ Z PAMIECI")
			print("X: %.4f" %self.X)
			print("Y: %.4f" %self.Y)
			print("Z: %.4f" %self.Z)
			print("KIN ODWROTNA")
			self.kinematykaOdwrotna(self.X, self.Y, self.Z)
			print("KIN PROSTA")
			self.kinematykaProsta(self.fi1, self.fi3, self.d3)
			
		else: 
			dioda_alarm.ChangeDutyCycle(0)
			serwo1.ChangeDutyCycle(fiToPWM(self.fi1))
			serwo2.ChangeDutyCycle(fiToPWM(self.fi3))
			print("Wypelnienie dla fi3: %.4f" %fiToPWM(self.fi3))
			serwo3.ChangeDutyCycle(d3ToPWM(self.d3))     #to musi byc regulowane
			print("Wyslano sygnaly sterujace")

#KONFIGURACJA WEJSC I WYJSC
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(4, GPIO.IN, pull_up_down=GPIO.PUD_UP)		#kierunek X up
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)		#kierunek X down
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)		#kierunek Y up
GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_UP)		#kierunek Y down
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_UP)		#kierunek Z up
GPIO.setup(9, GPIO.IN, pull_up_down=GPIO.PUD_UP)		#kierunek Z down
GPIO.setup(11, GPIO.IN, pull_up_down=GPIO.PUD_UP)		#sterowanie XYZ
GPIO.setup(0, GPIO.IN, pull_up_down=GPIO.PUD_UP)		#sterowanie JOINT
GPIO.setup(5, GPIO.OUT)	    #LED pozycji skrajnej
GPIO.setup(16, GPIO.OUT)	#serwo1: podstawa
GPIO.setup(20, GPIO.OUT)	#serwo2: ramie
GPIO.setup(21, GPIO.OUT)	#serwo3: 360 teleskop

dioda_alarm = GPIO.PWM(5, 1)
serwo1 = GPIO.PWM(16, 50)
serwo2 = GPIO.PWM(20, 50)
serwo3 = GPIO.PWM(21, 50)

#PARAMETRY STARTOWE VAR
fi1 = 0		#deg
fi3 = 60	#deg
d3 = 50 	#mm

#POZOSTALE WYMIARY CZLONOW
fi2 = 120	#deg
l1 = 40		#mm
l2 = 80		#mm
l4 = 100	#mm

#URUCHOMIENIE WYJSC
dioda_alarm.start(0)
serwo1.start(fiToPWM(fi1))		
serwo2.start(fiToPWM(fi3))
serwo3.start(0)		#regulacja
	
#PARAMETRY NARASTANIA WSPOLRZEDNYCH
dx = 5		#mm
dy = 5		#mm
dz = 5 		#mm
dfi1 = 5	#deg
dfi3 = 5	#deg
dd3 = 5		#mm
dt = 0.2	#s
btime = 200 #bouncetime ms

#OBSLUGA PRZERWAN OD PRZYCISKOW
def callbackButtons(channel):
	if channel == 4:
		while GPIO.input(channel) == GPIO.LOW:
			if chwytak.tryb_pracy == 'XYZ':
				print("\nRuch X+")
				chwytak.X += dx
				chwytak.sterowanieXYZ()
				sleep(dt)

			elif chwytak.tryb_pracy == 'JOINT':
				print("\nRuch fi1+")
				chwytak.fi1 += dfi1
				chwytak.sterowanieJOINT()
				sleep(dt)
				
	if channel == 17:
		while GPIO.input(channel) == GPIO.LOW:
			if chwytak.tryb_pracy == 'XYZ':
				print("\nRuch X-")
				chwytak.X -= dx
				chwytak.sterowanieXYZ()
				sleep(dt)

			elif chwytak.tryb_pracy == 'JOINT':
				print("\nRuch fi1-")
				chwytak.fi1 -= dfi1
				chwytak.sterowanieJOINT()
				sleep(dt)	
				
	if channel == 27:
		while GPIO.input(channel) == GPIO.LOW:
			if chwytak.tryb_pracy == 'XYZ':
				print("\nRuch Y+")
				chwytak.Y += dy
				chwytak.sterowanieXYZ()
				sleep(dt)

			elif chwytak.tryb_pracy == 'JOINT':
				print("\nRuch fi3+")
				chwytak.fi3 += dfi3
				chwytak.sterowanieJOINT()
				sleep(dt)	
				
	if channel == 22:
		while GPIO.input(channel) == GPIO.LOW:
			if chwytak.tryb_pracy == 'XYZ':
				print("\nRuch Y-")
				chwytak.Y -= dy
				chwytak.sterowanieXYZ()
				sleep(dt)

			elif chwytak.tryb_pracy == 'JOINT':
				print("\nRuch fi3-")
				chwytak.fi3 -= dfi3
				chwytak.sterowanieJOINT()
				sleep(dt)
				
	if channel == 10:
		while GPIO.input(channel) == GPIO.LOW:
			if chwytak.tryb_pracy == 'XYZ':
				print("\nRuch Z+")
				chwytak.Z += dz
				chwytak.sterowanieXYZ()
				sleep(dt)

			elif chwytak.tryb_pracy == 'JOINT':
				print("\nRuch d3+")
				chwytak.d3 += dd3
				chwytak.sterowanieJOINT()
				sleep(dt)
				
	if channel == 9:
		while GPIO.input(channel) == GPIO.LOW:
			if chwytak.tryb_pracy == 'XYZ':
				print("\nRuch Z-")
				chwytak.Z -= dz
				chwytak.sterowanieXYZ()
				sleep(dt)

			elif chwytak.tryb_pracy == 'JOINT':
				print("\nRuch d3-")
				chwytak.d3 -= dd3
				chwytak.sterowanieJOINT()
				sleep(dt)	
				
	if channel == 11:
		while GPIO.input(channel) == GPIO.LOW:
			chwytak.tryb_pracy = 'XYZ'	
			print("Tryb XYZ: ON")

		else:
			chwytak.tryb_pracy = 'none'
			print("Tryb XYZ: OFF")

	if channel == 0:
		if GPIO.input(channel) == GPIO.LOW:
			chwytak.tryb_pracy = 'JOINT'
			print("Tryb JOINT: ON")
		else:
			chwytak.tryb_pracy = 'none'
			print("Tryb JOINT: OFF")

#KONFIGURACJA PRZERWAN
GPIO.add_event_detect(4, GPIO.FALLING, callback=callbackButtons, bouncetime=btime)	
GPIO.add_event_detect(17, GPIO.FALLING, callback=callbackButtons, bouncetime=btime)
GPIO.add_event_detect(27, GPIO.FALLING, callback=callbackButtons, bouncetime=btime)
GPIO.add_event_detect(22, GPIO.FALLING, callback=callbackButtons, bouncetime=btime)	
GPIO.add_event_detect(10, GPIO.FALLING, callback=callbackButtons, bouncetime=btime)
GPIO.add_event_detect(9, GPIO.FALLING, callback=callbackButtons, bouncetime=btime)
GPIO.add_event_detect(11, GPIO.RISING, callback=callbackButtons, bouncetime=btime)
GPIO.add_event_detect(0, GPIO.FALLING, callback=callbackButtons, bouncetime=btime)

	
#POCZATEK WYKONYWANEGO PROGRAMU
print("\nPozycja startowa. Na podstawie wymiarow geometrycznych obliczane jest polozenie efektora (kinematyka prosta) a nastepnie sprawdzana jest poprawnosc obliczen  (kinematyka odwrotna). \n")
print("START:")
chwytak = Kinematyka(fi1,fi2,fi3,l1,l2,d3,l4, channelSetup(11,0))
sleep(1)
print("\nNacisnij przycisk aby poruszac manipulatorem.")	
	
#PETLA GLOWNA PROGRAMU
while True:

	try:
		sleep(2)
	except: KeyboardInterrupt

GPIO.cleanup()

