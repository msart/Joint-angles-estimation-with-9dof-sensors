
#quaternion initialization and beta
q0 = 1.0
q1 = 0.0
q2 = 0.0
q3 = 0.0
beta = 0.1

function invsqrt(x)
	return 1/sqrt(x)
end

function AHRS_update_IMU( gx,  gy,  gz,  ax,  ay,  az, q0, q1, q2, q3)

	#quaternion rate change
	qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
	qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
	qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
	qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

	#Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if !((ax == 0.0) && (ay == 0.0) && (az == 0.0)) 

		# Normalise accelerometer measurement
		recipNorm = invsqrt(ax * ax + ay * ay + az * az)
		ax *= recipNorm
		ay *= recipNorm
		az *= recipNorm   

		# Auxiliary variables to avoid repeated arithmetic
		two_q0 = 2.0 * q0
		two_q1 = 2.0 * q1
		two_q2 = 2.0 * q2
		two_q3 = 2.0 * q3
		four_q0 = 4.0 * q0
		four_q1 = 4.0 * q1
		four_q2 = 4.0 * q2
		eight_q1 = 8.0 * q1
		eight_q2 = 8.0 * q2
		q0q0 = q0 * q0
		q1q1 = q1 * q1
		q2q2 = q2 * q2
		q3q3 = q3 * q3

		# Gradient decent algorithm corrective step
		s0 = four_q0 * q2q2 + two_q2 * ax + four_q0 * q1q1 - two_q1 * ay
		s1 = four_q1 * q3q3 - two_q3 * ax + 4.0 * q0q0 * q1 - two_q0 * ay - four_q1 + eight_q1 * q1q1 + eight_q1 * q2q2 + four_q1 * az
		s2 = 4.0 * q0q0 * q2 + two_q0 * ax + four_q2 * q3q3 - two_q3 * ay - four_q2 + eight_q2 * q1q1 + eight_q2 * q2q2 + four_q2 * az
		s3 = 4.0 * q1q1 * q3 - two_q1 * ax + 4.0 * q2q2 * q3 - two_q2 * ay
		recipNorm = invsqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3) # normalise step magnitude
		s0 *= recipNorm
		s1 *= recipNorm
		s2 *= recipNorm
		s3 *= recipNorm

		# Apply feedback step
		qDot1 = qDot1 - beta * s0
		qDot2 = qDot2 - beta * s1
		qDot3 = qDot3 - beta * s2
		qDot4 = qDot4 - beta * s3
	end
	

	# Integrate rate of change of quaternion to yield quaternion
	q0 = q0 + qDot1 * (1.0 / 512.0)
	q1 = q1 +  qDot2 * (1.0 / 512.0)
	q2 = q2 +  qDot3 * (1.0 / 512.0)
	q3 = q3 +  qDot4 * (1.0 / 512.0)

	# Normalise quaternion
	recipNorm = invsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
	q0 = q0 * recipNorm
	q1 = q1 * recipNorm
	q2 = q2 * recipNorm
	q3 = q3 * recipNorm
	return q0, q1, q2, q3

end

function AHRS_update( gx,  gy,  gz,  ax,  ay,  az,  mx,  my,  mz, q0, q1, q2, q3)
	# Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if (mx == 0.0) && (my == 0.0) && (mz == 0.0) 
		q0, q1, q2, q3 = AHRS_update_IMU(gx, gy, gz, ax, ay, az, q0, q1, q2, q3)
		return q0, q1, q2, q3
	end
	

	# Rate of change of quaternion from gyroscope
	qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
	qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
	qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
	qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

	# Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if !((ax == 0.0) && (ay == 0.0) && (az == 0.0)) 

		# Normalise accelerometer measurement
		recipNorm = invsqrt(ax * ax + ay * ay + az * az)
		ax = ax * recipNorm
		ay = ay * recipNorm
		az = az * recipNorm  

		# Normalise magnetometer measurement
		recipNorm = invsqrt(mx * mx + my * my + mz * mz)
		mx = mx * recipNorm
		my = my * recipNorm
		mz = mz * recipNorm

		# Auxiliary variables to avoid repeated arithmetic
		two_q0mx = 2.0 * q0 * mx
		two_q0my = 2.0 * q0 * my
		two_q0mz = 2.0 * q0 * mz
		two_q1mx = 2.0 * q1 * mx
		two_q0 = 2.0 * q0
		two_q1 = 2.0 * q1
		two_q2 = 2.0 * q2
		two_q3 = 2.0 * q3
		two_q0q2 = 2.0 * q0 * q2
		two_q2q3 = 2.0 * q2 * q3
		q0q0 = q0 * q0
		q0q1 = q0 * q1
		q0q2 = q0 * q2
		q0q3 = q0 * q3
		q1q1 = q1 * q1
		q1q2 = q1 * q2
		q1q3 = q1 * q3
		q2q2 = q2 * q2
		q2q3 = q2 * q3
		q3q3 = q3 * q3

		# Reference direction of Earth's magnetic field
		hx = mx * q0q0 - two_q0my * q3 + two_q0mz * q2 + mx * q1q1 + two_q1 * my * q2 + two_q1 * mz * q3 - mx * q2q2 - mx * q3q3
		hy = two_q0mx * q3 + my * q0q0 - two_q0mz * q1 + two_q1mx * q2 - my * q1q1 + my * q2q2 + two_q2 * mz * q3 - my * q3q3
		two_bx = sqrt(hx * hx + hy * hy)
		two_bz = -two_q0mx * q2 + two_q0my * q1 + mz * q0q0 + two_q1mx * q3 - mz * q1q1 + two_q2 * my * q3 - mz * q2q2 + mz * q3q3
		four_bx = 2.0 * two_bx
		four_bz = 2.0 * two_bz

		# Gradient decent algorithm corrective step
		s0 = -two_q2 * (2.0 * q1q3 - two_q0q2 - ax) + two_q1 * (2.0 * q0q1 + two_q2q3 - ay) - two_bz * q2 * (two_bx * (0.5f - q2q2 - q3q3) + two_bz * (q1q3 - q0q2) - mx) + (-two_bx * q3 + two_bz * q1) * (two_bx * (q1q2 - q0q3) + two_bz * (q0q1 + q2q3) - my) + two_bx * q2 * (two_bx * (q0q2 + q1q3) + two_bz * (0.5f - q1q1 - q2q2) - mz)
		s1 = two_q3 * (2.0 * q1q3 - two_q0q2 - ax) + two_q0 * (2.0 * q0q1 + two_q2q3 - ay) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + two_bz * q3 * (two_bx * (0.5f - q2q2 - q3q3) + two_bz * (q1q3 - q0q2) - mx) + (two_bx * q2 + two_bz * q0) * (two_bx * (q1q2 - q0q3) + two_bz * (q0q1 + q2q3) - my) + (two_bx * q3 - four_bz * q1) * (two_bx * (q0q2 + q1q3) + two_bz * (0.5f - q1q1 - q2q2) - mz)
		s2 = -two_q0 * (2.0 * q1q3 - two_q0q2 - ax) + two_q3 * (2.0 * q0q1 + two_q2q3 - ay) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-four_bx * q2 - two_bz * q0) * (two_bx * (0.5f - q2q2 - q3q3) + two_bz * (q1q3 - q0q2) - mx) + (two_bx * q1 + two_bz * q3) * (two_bx * (q1q2 - q0q3) + two_bz * (q0q1 + q2q3) - my) + (two_bx * q0 - four_bz * q2) * (two_bx * (q0q2 + q1q3) + two_bz * (0.5f - q1q1 - q2q2) - mz)
		s3 = two_q1 * (2.0 * q1q3 - two_q0q2 - ax) + two_q2 * (2.0 * q0q1 + two_q2q3 - ay) + (-four_bx * q3 + two_bz * q1) * (two_bx * (0.5f - q2q2 - q3q3) + two_bz * (q1q3 - q0q2) - mx) + (-two_bx * q0 + two_bz * q2) * (two_bx * (q1q2 - q0q3) + two_bz * (q0q1 + q2q3) - my) + two_bx * q1 * (two_bx * (q0q2 + q1q3) + two_bz * (0.5f - q1q1 - q2q2) - mz)
		recipNorm = invsqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3) # normalise step magnitude
		s0 = s0 * recipNorm
		s1 = s1 * recipNorm
		s2 = s2 * recipNorm
		s3 = s3 * recipNorm

		# Apply feedback step
		qDot1 = qDot1 - beta * s0
		qDot2 = qDot2 - beta * s1
		qDot3 = qDot3 - beta * s2
		qDot4 = qDot4 - beta * s3
	end
	

	# Integrate rate of change of quaternion to yield quaternion
	q0 = q0 + qDot1 * (1.0 / 512.0)
	q1 = q1 + qDot2 * (1.0 / 512.0)
	q2 = q2 + qDot3 * (1.0 / 512.0)
	q3 = q3 + qDot4 * (1.0 / 512.0)

	# Normalise quaternion
	recipNorm = invsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
	q0 = q0 * recipNorm
	q1 = q1 * recipNorm
	q2 = q2 * recipNorm
	q3 = q3 * recipNorm
	return q0, q1, q2, q3
end



gx = 0.0
gy = 0.0
gz = 0.0
ax = 1.0
ay = 1.0
az = 1.0
mx = 0.0
my = 0.0
mz = 0.0
x = 1 
y = 0
print(q0," ", q1," ", q2," ", q3, " \n")

for i = 0:100 
    q0, q1, q2, q3 = AHRS_update(gx, gy, gz, ax, ay, az, mx, my, mz, q0, q1, q2, q3)
    print(q0," ", q1," ", q2," ", q3, " \n")
end
    
