#!/usr/bin/python
# -*- coding: iso-8859-1 -*-

import serial

sensor_1 = serial.Serial('/dev/ttyUSB0', 9600)
x = [1, 2, 3]
#sensor_2 = serial.Serial('/dev/ttyUSB1', 9600) 
print("sensor_2 ", sensor_1.readline())
print("sensor_2 ", sensor_1.readline())
print("sensor_2 ", sensor_1.readline())
print("sensor_2 ", sensor_1.readline())

i = 0
while True:
	s = sensor_1.readline()
	s_s = s.split()
	#heading.append(float(s_s[0].decode('UTF-8')))
	#pitch.append(float(s_s[1].decode('UTF-8')))
	#yaw.append(float(s_s[2].decode('UTF-8')))
	x[0] = float(s_s[0].decode('UTF-8'))
	x[1] = float(s_s[1].decode('UTF-8'))
	x[2] = float(s_s[2].decode('UTF-8'))
	print(x[0], x[1], x[2])
	#print("sensor_2 ", sensor_2.readline())
	
