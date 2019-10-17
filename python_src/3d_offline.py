#!/usr/bin/python
# -*- coding: iso-8859-1 -*-

import sys
import AHRS_Madgwick

from time import sleep


def quaternion(q0, q1, q2, q3):
	q = []

	q.append(q0)
	q.append(q1)
	q.append(q2)
	q.append(q3)

	return q

def conjugate_quaternion(q):
	conjugate_q = []

	conjugate_q.append(q[0])
	conjugate_q.append(-q[1])
	conjugate_q.append(-q[2])
	conjugate_q.append(-q[3])

	return conjugate_q

def diff_quaternion(q1, q2):
	diff_q= []

	q2 = conjugate_quaternion(q2)
	diff_q.append(q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3])
	diff_q.append(q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2])
	diff_q.append(q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1])
	diff_q.append(q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0])

	return diff_q


def mean_filter(signal):
	sum = 0.0
	for i in signal:
		sum += i
	return sum / len(signal) 

def filtered_angle(sample, samples_vector):
	if len(samples_vector) < 5:
		samples_vector.append(sample)
	else:
		del samples_vector[0]
		samples_vector.append(sample)
	return mean_filter(samples_vector)



def parse_sensor_data(msg):
	s_s = msg.split()

	gx = float(s_s[0][2:])

	gy = float(s_s[1])
	gz = float(s_s[2])
	ax = float(s_s[3])
	ay = float(s_s[4])
	az = float(s_s[5])
	mx = float(s_s[6])
	my = float(s_s[7])
	mz = float(s_s[8][:-6])

	return gx, gy, gz, ax, ay, az, mx, my, mz


def joint_angle_estimation(sensor1_pitch, sensor2_pitch):
	return -sensor2_pitch - sensor1_pitch


def main():


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

	# while True:
	# for i in range(1,10)
	with open(sys.argv[1], 'r') as f:
		while True:
			#sensor 1 reading msg
			msg = f.readline()
			if not msg: break
			sensor1_gx, sensor1_gy, sensor1_gz, sensor1_ax, sensor1_ay, sensor1_az, sensor1_mx, sensor1_my, sensor1_mz = parse_sensor_data(msg)
			# print(sensor1_gx, sensor1_gy, sensor1_gz, sensor1_ax, sensor1_ay, sensor1_az, sensor1_mx, sensor1_my, sensor1_mz)

			#sensor 1 updating
			for i in range(100):
				sensor1_q0, sensor1_q1, sensor1_q2, sensor1_q3 = AHRS_Madgwick.update(sensor1_gx, sensor1_gy, sensor1_gz, sensor1_ax, sensor1_ay, sensor1_az, sensor1_mx, sensor1_my, sensor1_mz, sensor1_q0, sensor1_q1, sensor1_q2, sensor1_q3)
			q1 = quaternion(sensor1_q0, sensor1_q1, sensor1_q2, sensor1_q3)
			# print(sensor1_q0, sensor1_q1, sensor1_q2, sensor1_q3)

			#sensor 1 calculating angles
			# sensor1_roll, sensor1_pitch, sensor1_yaw = AHRS_Madgwick.compute_angles(sensor1_q0, sensor1_q1, sensor1_q2, sensor1_q3)
			# print(sensor1_roll, sensor1_pitch, sensor1_yaw)

			#sensor 1 reading msg
			msg = f.readline()
			if not msg: break
			sensor2_gx, sensor2_gy, sensor2_gz, sensor2_ax, sensor2_ay, sensor2_az, sensor2_mx, sensor2_my, sensor2_mz = parse_sensor_data(msg)

			#sensor 2 updating
			for i in range(100):
				sensor2_q0, sensor2_q1, sensor2_q2, sensor2_q3 = AHRS_Madgwick.update(sensor2_gx, sensor2_gy, sensor2_gz, sensor2_ax, sensor2_ay, sensor2_az, sensor2_mx, sensor2_my, sensor2_mz, sensor2_q0, sensor2_q1, sensor2_q2, sensor2_q3)
			q2 = quaternion(sensor2_q0, sensor2_q1, sensor2_q2, sensor2_q3)

			# print(sensor2_q0, sensor2_q1, sensor2_q2, sensor2_q3)

			#sensor 2 calculating angles
			# sensor2_roll, sensor2_pitch, sensor2_yaw = AHRS_Madgwick.compute_angles(sensor2_q0, sensor2_q1, sensor2_q2, sensor2_q3)
			# print(sensor2_roll, sensor2_pitch, sensor2_yaw)


			# angle = joint_angle_estimation(sensor1_pitch, sensor2_pitch)
			# f_angle = filtered_angle(angle, samples)

			diff = diff_quaternion(q2, q1)
			roll, pitch, yaw = AHRS_Madgwick.compute_angles(diff[0], diff[1], diff[2], diff[3])


			print("roll:", roll, "pitch:", pitch, "yaw:", yaw)

			#normalizing and sending message to puredata
			# angle = angle / 180.0
			# message = str(angle) + ";"
			# s.send(message.encode('utf-8'))

			# sleep(0.01)


main()