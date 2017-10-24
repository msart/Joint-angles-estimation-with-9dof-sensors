#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>

Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);


// Mag calibration values are calculated via ahrs_calibration.
// These values must be determined for each baord/environment.
// See the image in this sketch folder for the values used
// below.

// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { 11.20F, -15.05F, 59.89F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  0.978,  0.008,  0.016 },
                                    {  0.008,  0.940, 0.040 },
                                    {  0.016, 0.040,  1.090 } };

float mag_field_strength        = 22.32F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };


void setup()
{
  Serial.begin(9600);


  // Initialize the sensors.
  if(!gyro.begin())
  {
    /* There was a problem detecting the gyro ... check your connections */
    Serial.println("Ooops, no gyro detected ... Check your manda as d�vidas! = ) djwiring!");
    while(1);
  }

  if(!accelmag.begin(ACCEL_RANGE_4G))
  {
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while(1);
  }

}

void loop(void)
{
  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;

  // Get new data samples
  gyro.getEvent(&gyro_event);
  accelmag.getEvent(&accel_event, &mag_event);


  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Apply gyro zero-rate error compensation
  float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
  float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
  float gz = gyro_event.gyro.z + gyro_zero_offsets[2];

  // The filter library expects gyro data in degrees/s, but adafruit sensor
  // uses rad/s so we need to convert them first (or adapt the filter lib
  // where they are being converted)
  gx *= 57.2958F;
  gy *= 57.2958F;
  gz *= 57.2958F;
	
	//Serial.print("Gyro_X: "); 
	Serial.print(gx); Serial.print(" ");
  //Serial.print("Gyro_Y: "); 
  Serial.print(gy); Serial.print(" ");
  //Serial.print("Gyro_Z: "); 
  Serial.print(gz); Serial.print(" ");
	//Serial.print("Accel_X: "); 
	Serial.print(accel_event.acceleration.x); Serial.print(" ");
  //Serial.print("Accel_Y: "); 
  Serial.print(accel_event.acceleration.y); Serial.print(" ");
  //Serial.print("Accel_Z: "); 
  Serial.print(accel_event.acceleration.z); Serial.print(" ");
	//Serial.print("Mag_X: "); 
	Serial.print(mx); Serial.print(" ");
  //Serial.print("Mag_Y: "); 
  Serial.print(my); Serial.print(" ");
  //Serial.print("Mag_Z: "); 
  Serial.println(mz);


  delay(10);
}
