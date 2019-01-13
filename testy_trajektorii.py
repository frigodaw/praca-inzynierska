#IMPORT KONIECZNYCH MODULOW I PAKIETOW
from collections import deque
import numpy as np
import argparse
from imutils.video import VideoStream
import imutils
import time
import cv2

#PARSOWANIE ARGUMENTOW
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
ap.add_argument("-r", "--record_video",
	help="path to the video file to write (optional)")
ap.add_argument("-t", "--record_track", action='store_true',
	help="record video with the tracking result (valid only if -r is given, optional)", required=False)
args = vars(ap.parse_args())

#DEFINICJA DOLNEGO I GORNEGO ZAKRESU KOLORU CZERWONEGO
greenLower = (0, 105, 130)
greenUpper = (7, 255, 255)

#INICJALIZACJA LISTY SLEDZONYCH PUNKTOW
pts = deque(maxlen=args["buffer"])

#VIDEO
record_video = args.get("record_video", False)
record_track = args.get("record_track", False)
video_writer = None

#WYBOR MIEDZY KAMERA A PLIKIEM VIDEO
from_vfile = args.get("video", False)
if not from_vfile:
	camera = VideoStream(src=0).start()
	time.sleep(2.0)
else:
	video_file = cv2.VideoCapture(args["video"])
	
#OTWARCIE PLIKU DO ZAPISU
file = open('data.txt','w')
file.write("n		X		Y\n")
n = 0

#PETLA GLOWNA
while True:
	#PRZECHWYCENIE KLATKI
	if from_vfile:
		(grabbed, frame) = video_file.read()

		#W PRZYPADKU BRAKU KLATEK TO KONIEC VIDEO
		if not grabbed:
			break
	else:
		frame = camera.read()
	
	frame = imutils.resize(frame, width=600)
	if record_video:
		if video_writer is None: #STWORZ INTERFEJS
			(h, w) = frame.shape[:2]
			fourcc = cv2.VideoWriter_fourcc(*'MJPG')
			video_writer = cv2.VideoWriter(args["record_video"], fourcc, 20, (w,h), True)

		if not record_track:
			video_writer.write(frame)
			
	#PROGOWANIE
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	#ODNAJDOWANIE KONTUROW I WSPOLRZEDNYCH SRODKA
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)[-2]
	center = None

	if len(cnts) > 0:
		#ODNAJDUJE NAJWIEKSZY OBSZAR ZADANEJ BARWY
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		if radius > 3:
			#RYSOWANIE OKREGU WOKOL OBRYSU
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)

	pts.appendleft(center)

	#OPERACJE NAD ZBIOREM SLEDZONYCH PUNKTOW
	for i in xrange(1, len(pts)):
		if pts[i - 1] is None or pts[i] is None:
			continue

		thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
		cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
		
	#EFEKT RUCHU SMUGI
	cv2.putText(frame, "X: {}, Y: {}, n: {}".format(x, y, n),
		(10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
		0.35, (0, 0, 255), 1)
	
	#ZAPIS DO PLIKU
	buf = ""
	buf += str(n)
	buf += "		"
	buf += str(round(x,2))
	buf += "		"
	buf += str(round(y,2))
	buf += '\n'
	file.write(buf)
	buf = ""
	n+=1
	
	#WYSWIETLANIE OKNA
	cv2.imshow("Frame", frame)

	if record_video and record_track:
		video_writer.write(frame)
	
	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):
		break

#CLEANUP
if not from_vfile:
	if hasattr(camera, 'release'):
		camera.release()
else:
	video_file.release()

if video_writer is not None:
	video_writer.release()

file.close()
cv2.destroyAllWindows()
