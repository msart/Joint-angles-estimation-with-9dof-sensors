#!/usr/bin/python
# -*- coding: iso-8859-1 -*-

import serial
import AHRS_Madgwick


def main():
	q0 = 1.0
	q1 = 0.0
	q2 = 0.0
	q3 = 0.0

	ax = 0.0
	ay = 0.0
	az = 0.0
	gx = 0.0
	gy = 0.0
	gz = 0.0
	mx = 0.0
	my = 0.0
	mz = 0.0

	for i in range(1,10):
		q0,q1,q2,q3 = AHRS_Madgwick.update(ax,ay,az,gx,gy,gz,mx,my,mz,q0,q1,q2,q3)
		print(q0,q1,q2,q3)
		roll, pitch, yaw = AHRS_Madgwick.compute_angles(q0, q1, q2, q3)
		print(roll, pitch, yaw)
		ax += 1.0
		ay += 1.0
		az += 1.0
		gx += 1.0
		gy += 1.0
		gz += 1.0
		mx += 1.0
		my += 1.0
		mz += 1.0
	# sensor_1 = serial.Serial('/dev/ttyUSB0', 9600) 
	# sensor_2 = serial.Serial('/dev/ttyUSB1', 9600) 

	# i = 0
	# while i < 10:
	# 	print("sensor_1 ", sensor_1.readline())

	# 	print("sensor_2 ", sensor_2.readline())
	# 	i += 1


main()