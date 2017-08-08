sampleFreq = 512.0
twoKp = 1.0											
twoKi = 0.0							
q0 = 1.0
q1 = 0.0
q2 = 0.0
q3 = 0.0
integralFBx = 0.0
integralFBy = 0.0
integralFBz = 0.0	

function invsqrt(x)
	return 1/sqrt(x)
end

void AHRS_update_IMU(gx, gy, gz, ax, ay, az, q0, q1, q2, q3) {

	if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

		norm = invsqrt(ax * ax + ay * ay + az * az)
		ax = ax * norm
		ay = ay * norm
		az = az * norm      

		halfvx = q1 * q3 - q0 * q2
		halfvy = q0 * q1 + q2 * q3
		halfvz = q0 * q0 - 0.5 + q3 * q3
	
		halfex = (ay * halfvz - az * halfvy)
		halfey = (az * halfvx - ax * halfvz)
		halfez = (ax * halfvy - ay * halfvx)

		if(twoKi > 0.0) {
			integralFBx = integralFBx + twoKi * halfex * (1.0 / sampleFreq)
			integralFBy = integralFBy + twoKi * halfey * (1.0 / sampleFreq)
			integralFBz = integralFBz + twoKi * halfez * (1.0 / sampleFreq)
			gx = gx + integralFBx	
			gy = gy + integralFBy
			gz = gz + integralFBz
		}
		else {
			integralFBx = 0.0	
			integralFBy = 0.0
			integralFBz = 0.0
		}

		gx = gx + twoKp * halfex
		gy = gy + twoKp * halfey
		gz = gz + twoKp * halfez
	}
	
	gx = gx * (0.5 * (1.0 / sampleFreq))
	gy = gy * (0.5 * (1.0 / sampleFreq))
	gz = gz * (0.5 * (1.0 / sampleFreq))
	qa = q0
	qb = q1
	qc = q2
	q0 = q0 + (-qb * gx - qc * gy - q3 * gz)
	q1 = q1 + (qa * gx + qc * gz - q3 * gy)
	q2 = q2 + (qa * gy - qb * gz + q3 * gx)
	q3 = q3 + (qa * gz + qb * gy - qc * gx) 
	
	norm = invsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
	q0 = q0 * norm
	q1 = q1 * norm
	q2 = q2 * norm
	q3 = q3 * norm
}


void AHRS_update(gx, gy, gz, ax, ay, az, mx, my, mz, q0, q1, q2, q3) {

	if((mx == 0.0) && (my == 0.0) && (mz == 0.0)) {
		AHRS_update_IMU(gx, gy, gz, ax, ay, az, q0, q1, q2, q3)
		return
	}

	if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

		norm = invsqrt(ax * ax + ay * ay + az * az)
		ax = ax * norm
		ay = ay * norm
		az = az * norm     

		norm = invsqrt(mx * mx + my * my + mz * mz)
		mx = mx * norm
		my = my * norm
		mz = mz * norm   

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

        hx = 2.0 * (mx * (0.5 - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2))
        hy = 2.0 * (mx * (q1q2 + q0q3) + my * (0.5 - q1q1 - q3q3) + mz * (q2q3 - q0q1))
        bx = sqrt(hx * hx + hy * hy)
        bz = 2.0 * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5 - q1q1 - q2q2))

		halfvx = q1q3 - q0q2
		halfvy = q0q1 + q2q3
		halfvz = q0q0 - 0.5 + q3q3
        halfwx = bx * (0.5 - q2q2 - q3q3) + bz * (q1q3 - q0q2)
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3)
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5 - q1q1 - q2q2)  
	
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy)
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz)
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx)

		if(twoKi > 0.0) {
			integralFBx = integralFBx + twoKi * halfex * (1.0 / sampleFreq)	
			integralFBy = integralFBy + twoKi * halfey * (1.0 / sampleFreq)
			integralFBz = integralFBz + twoKi * halfez * (1.0 / sampleFreq)
			gx = gx + integralFBx	
			gy = gy + integralFBy
			gz = gz + integralFBz
		}
		else {
			integralFBx = 0.0	
			integralFBy = 0.0
			integralFBz = 0.0
		}

		gx = gx + twoKp * halfex
		gy = gy + twoKp * halfey
		gz = gz + twoKp * halfez
	}
	
	gx = gx * (0.5 * (1.0 / sampleFreq))	
	gy = gy * (0.5 * (1.0 / sampleFreq))
	gz = gz * (0.5 * (1.0 / sampleFreq))
	qa = q0
	qb = q1
	qc = q2
	q0 = q0 + (-qb * gx - qc * gy - q3 * gz)
	q1 = q1 + (qa * gx + qc * gz - q3 * gy)
	q2 = q2 + (qa * gy - qb * gz + q3 * gx)
	q3 = q3 + (qa * gz + qb * gy - qc * gx) 
	
	norm = invsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
	q0 = q0 * norm
	q1 = q1 * norm
	q2 = q2 * norm
	q3 = q3 * norm
}

