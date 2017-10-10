from math import sqrt
from math import atan2
from math import asin


beta = 0.1

def invsqrt(number):
    return number ** -0.5

def update_IMU( gx,  gy,  gz,  ax,  ay,  az, q0, q1, q2, q3):	


	gx = gx * 0.0174533
	gy = gy * 0.0174533
	gz = gz * 0.0174533	

	qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
	qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
	qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
	qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)	

	if not ((ax == 0.0) and (ay == 0.0) and (az == 0.0)): 	

		norm = invsqrt(ax * ax + ay * ay + az * az)
		ax = ax * norm
		ay = ay * norm
		az = az * norm  	

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

		s0 = four_q0 * q2q2 + two_q2 * ax + four_q0 * q1q1 - two_q1 * ay
		s1 = four_q1 * q3q3 - two_q3 * ax + 4.0 * q0q0 * q1 - two_q0 * ay - four_q1 + eight_q1 * q1q1 + eight_q1 * q2q2 + four_q1 * az
		s2 = 4.0 * q0q0 * q2 + two_q0 * ax + four_q2 * q3q3 - two_q3 * ay - four_q2 + eight_q2 * q1q1 + eight_q2 * q2q2 + four_q2 * az
		s3 = 4.0 * q1q1 * q3 - two_q1 * ax + 4.0 * q2q2 * q3 - two_q2 * ay
		norm = invsqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3) 
		print(s0," ", s1," ", s2," ", s3, " ", norm, " \n")
		s0 = s0 * norm
		s1 = s1 * norm
		s2 = s2 * norm
		s3 = s3 * norm	

		qDot1 = qDot1 - beta * s0
		qDot2 = qDot2 - beta * s1
		qDot3 = qDot3 - beta * s2
		qDot4 = qDot4 - beta * s3

	#print(norm ,"\n")
	#print(s0," ", s1," ", s2," ", s3, " \n")
	#print(qDot1," ", qDot2," ", qDot3," ", qDot4, " \n")
	q0 = q0 +  qDot1 * (1.0 / 512.0)
	q1 = q1 +  qDot2 * (1.0 / 512.0)
	q2 = q2 +  qDot3 * (1.0 / 512.0)
	q3 = q3 +  qDot4 * (1.0 / 512.0)	

	norm = invsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
	q0 = q0 * norm
	q1 = q1 * norm
	q2 = q2 * norm
	q3 = q3 * norm
	return q0, q1, q2, q3	


def update( gx,  gy,  gz,  ax,  ay,  az,  mx,  my,  mz, q0, q1, q2, q3):
	# Usa IMU para o caso se a medição do magnetometro ser invalida
	if (mx == 0.0) and (my == 0.0) and (mz == 0.0) :
		q0, q1, q2, q3 = update_IMU(gx, gy, gz, ax, ay, az, q0, q1, q2, q3)
		return q0, q1, q2, q3
	# De graus/sec pra rad/sec
	gx = gx * 0.0174533
	gy = gy * 0.0174533
	gz = gz * 0.0174533

	# Taxa de variação do quaternion pelo giroscopio
	qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
	qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
	qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
	qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

	if not ((ax == 0.0) and (ay == 0.0) and (az == 0.0)):

		# Normalizando dados do acelerometro
		norm = invsqrt(ax * ax + ay * ay + az * az)
		ax = ax * norm
		ay = ay * norm
		az = az * norm  

		# Normaliza magnetometro
		norm = invsqrt(mx * mx + my * my + mz * mz)
		mx = mx * norm
		my = my * norm
		mz = mz * norm

		# Pra nao repetir calculos
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

		# compensação da direção do campo magnetico
		hx = mx * q0q0 - two_q0my * q3 + two_q0mz * q2 + mx * q1q1 + two_q1 * my * q2 + two_q1 * mz * q3 - mx * q2q2 - mx * q3q3
		hy = two_q0mx * q3 + my * q0q0 - two_q0mz * q1 + two_q1mx * q2 - my * q1q1 + my * q2q2 + two_q2 * mz * q3 - my * q3q3
		two_bx = sqrt(hx * hx + hy * hy)
		two_bz = -two_q0mx * q2 + two_q0my * q1 + mz * q0q0 + two_q1mx * q3 - mz * q1q1 + two_q2 * my * q3 - mz * q2q2 + mz * q3q3
		four_bx = 2.0 * two_bx
		four_bz = 2.0 * two_bz

		# Gradiente descendente
		s0 = -two_q2 * (2.0 * q1q3 - two_q0q2 - ax) + two_q1 * (2.0 * q0q1 + two_q2q3 - ay) - two_bz * q2 * (two_bx * (0.5 - q2q2 - q3q3) + two_bz * (q1q3 - q0q2) - mx) + (-two_bx * q3 + two_bz * q1) * (two_bx * (q1q2 - q0q3) + two_bz * (q0q1 + q2q3) - my) + two_bx * q2 * (two_bx * (q0q2 + q1q3) + two_bz * (0.5 - q1q1 - q2q2) - mz)
		s1 = two_q3 * (2.0 * q1q3 - two_q0q2 - ax) + two_q0 * (2.0 * q0q1 + two_q2q3 - ay) - 4.0 * q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + two_bz * q3 * (two_bx * (0.5 - q2q2 - q3q3) + two_bz * (q1q3 - q0q2) - mx) + (two_bx * q2 + two_bz * q0) * (two_bx * (q1q2 - q0q3) + two_bz * (q0q1 + q2q3) - my) + (two_bx * q3 - four_bz * q1) * (two_bx * (q0q2 + q1q3) + two_bz * (0.5 - q1q1 - q2q2) - mz)
		s2 = -two_q0 * (2.0 * q1q3 - two_q0q2 - ax) + two_q3 * (2.0 * q0q1 + two_q2q3 - ay) - 4.0 * q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-four_bx * q2 - two_bz * q0) * (two_bx * (0.5 - q2q2 - q3q3) + two_bz * (q1q3 - q0q2) - mx) + (two_bx * q1 + two_bz * q3) * (two_bx * (q1q2 - q0q3) + two_bz * (q0q1 + q2q3) - my) + (two_bx * q0 - four_bz * q2) * (two_bx * (q0q2 + q1q3) + two_bz * (0.5 - q1q1 - q2q2) - mz)
		s3 = two_q1 * (2.0 * q1q3 - two_q0q2 - ax) + two_q2 * (2.0 * q0q1 + two_q2q3 - ay) + (-four_bx * q3 + two_bz * q1) * (two_bx * (0.5 - q2q2 - q3q3) + two_bz * (q1q3 - q0q2) - mx) + (-two_bx * q0 + two_bz * q2) * (two_bx * (q1q2 - q0q3) + two_bz * (q0q1 + q2q3) - my) + two_bx * q1 * (two_bx * (q0q2 + q1q3) + two_bz * (0.5 - q1q1 - q2q2) - mz)
		
		#Normalizando
		norm = invsqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
		s0 = s0 * norm
		s1 = s1 * norm
		s2 = s2 * norm
		s3 = s3 * norm

		# passo do feedback
		qDot1 = qDot1 - beta * s0
		qDot2 = qDot2 - beta * s1
		qDot3 = qDot3 - beta * s2
		qDot4 = qDot4 - beta * s3
	
	# aplicando no quaterinon
	q0 = q0 + qDot1 * (1.0 / 512.0)
	q1 = q1 + qDot2 * (1.0 / 512.0)
	q2 = q2 + qDot3 * (1.0 / 512.0)
	q3 = q3 + qDot4 * (1.0 / 512.0)

	# Normalizando
	norm = invsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
	q0 = q0 * norm
	q1 = q1 * norm
	q2 = q2 * norm
	q3 = q3 * norm
	return q0, q1, q2, q3



def compute_angles(q0, q1, q2, q3):
	roll = atan2(q0*q1 + q2*q3, 0.5 - q1*q1 - q2*q2);
	pitch = asin(-2.0 * (q1*q3 - q0*q2));
	yaw = atan2(q1*q2 + q0*q3, 0.5 - q2*q2 - q3*q3);
	return roll, pitch, yaw