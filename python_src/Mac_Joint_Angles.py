#!/usr/bin/python
# -*- coding: iso-8859-1 -*-

import serial
import AHRS_Madgwick
import socket
from time import sleep



def parse_sensor_data(msg):
	s_s = msg.split()

	gx = float(s_s[0].decode('UTF-8'))
	gy = float(s_s[1].decode('UTF-8'))
	gz = float(s_s[2].decode('UTF-8'))
	ax = float(s_s[3].decode('UTF-8'))
	ay = float(s_s[4].decode('UTF-8'))
	az = float(s_s[5].decode('UTF-8'))
	mx = float(s_s[6].decode('UTF-8'))
	my = float(s_s[7].decode('UTF-8'))
	mz = float(s_s[8].decode('UTF-8'))

	return gx, gy, gz, ax, ay, az, mx, my, mz


def joint_angle_estimation(sensor1_pitch, sensor2_pitch):
	return -sensor2_pitch - sensor1_pitch


def main():

	# s = socket.socket()
	# host = socket.gethostname()
	# port = 3000
	# s.connect((host, port))

	sensor_1 = serial.Serial('/dev/ttyUSB0', 9600) 
	sensor_2 = serial.Serial('/dev/ttyUSB1', 9600) 

	for i in range(1,10):
		sensor_1.readline()
		sensor_2.readline()

	#forearm sensor
	sensor1_q0 = 1.0
	sensor1_q1 = 0.0
	sensor1_q2 = 0.0
	sensor1_q3 = 0.0

	#arm sensor
	sensor2_q0 = 1.0
	sensor2_q1 = 0.0
	sensor2_q2 = 0.0
	sensor2_q3 = 0.0

	while True:
	# for i in range(1,10):

		#sensor 1 reading msg
		msg = sensor_1.readline()
		sensor1_gx, sensor1_gy, sensor1_gz, sensor1_ax, sensor1_ay, sensor1_az, sensor1_mx, sensor1_my, sensor1_mz = parse_sensor_data(msg)
		# print(sensor1_gx, sensor1_gy, sensor1_gz, sensor1_ax, sensor1_ay, sensor1_az, sensor1_mx, sensor1_my, sensor1_mz)

		#sensor 1 updating
		sensor1_q0, sensor1_q1, sensor1_q2, sensor1_q3 = AHRS_Madgwick.update(sensor1_gx, sensor1_gy, sensor1_gz, sensor1_ax, sensor1_ay, sensor1_az, sensor1_mx, sensor1_my, sensor1_mz, sensor1_q0, sensor1_q1, sensor1_q2, sensor1_q3)
		# print(sensor1_q0, sensor1_q1, sensor1_q2, sensor1_q3)

		#sensor 1 calculating angles
		sensor1_roll, sensor1_pitch, sensor1_yaw = AHRS_Madgwick.compute_angles(sensor1_q0, sensor1_q1, sensor1_q2, sensor1_q3)
		# print(sensor1_roll, sensor1_pitch, sensor1_yaw)

		#sensor 1 reading msg
		msg = sensor_2.readline()
		sensor2_gx, sensor2_gy, sensor2_gz, sensor2_ax, sensor2_ay, sensor2_az, sensor2_mx, sensor2_my, sensor2_mz = parse_sensor_data(msg)

		#sensor 2 updating
		sensor2_q0, sensor2_q1, sensor2_q2, sensor2_q3 = AHRS_Madgwick.update(sensor2_gx, sensor2_gy, sensor2_gz, sensor2_ax, sensor2_ay, sensor2_az, sensor2_mx, sensor2_my, sensor2_mz, sensor2_q0, sensor2_q1, sensor2_q2, sensor2_q3)
		# print(sensor2_q0, sensor2_q1, sensor2_q2, sensor2_q3)

		#sensor 2 calculating angles
		sensor2_roll, sensor2_pitch, sensor2_yaw = AHRS_Madgwick.compute_angles(sensor2_q0, sensor2_q1, sensor2_q2, sensor2_q3)
		# print(sensor2_roll, sensor2_pitch, sensor2_yaw)


		angle = joint_angle_estimation(sensor1_pitch, sensor2_pitch)
		print("Angle estimation: ", sensor1_pitch, sensor2_pitch, angle)


		#normalizing and sending message to puredata
		angle = angle / 180.0
		message = str(angle) + ";"
		# s.send(message.encode('utf-8'))

		# sleep(0.01)


main()