#!/usr/bin/python
# -*- coding: iso-8859-1 -*-

import serial

sensor_1 = serial.Serial('/dev/ttyUSB0', 9600) 
sensor_2 = serial.Serial('/dev/ttyUSB1', 9600) 

i = 0
while i < 10:
	print("sensor_1 ", sensor_1.readline())

	print("sensor_2 ", sensor_2.readline())
	i += 1
