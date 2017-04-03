/*************************************************
** FILE: MPU9250_Functions
** This file contains some MPU 9250 (HW specific)
** functions. Specifically, for initializing and 
** reading the sensor registeres
**************************************************/



/*************************************************
** Read_Sensors 
** This function reads the sensor registers and
** assigns them to the global input vectors
*/
void Read_Sensors()
{
  /* Set the initial accel and gyro vectors */
  imu.updateAccel();
  accel[0] = imu.ax;
  accel[1] = imu.ay;
  accel[2] = imu.az;
  imu.updateGyro();
  gyro[0] = imu.gx;
  gyro[1] = imu.gy;
  gyro[2] = imu.gz;
}


/*************************************************
** Init_IMU
** This function set the IMU parameters
** This includes things like the SR and the 
** internal LPF corner. 
*/
bool Init_IMU(void)
{
  /* Set up MPU-9250 interrupt input (active-low) */
  pinMode(MPU9250_INT_PIN, INPUT_PULLUP);
  
  /* imu.begin() should return 0 on success. Will initialize
  ** I2C bus, and reset MPU-9250 to defaults */
  if (imu.begin() != INV_SUCCESS) { return false; }

  /* Initiate accel and gyro sensors only */
  imu.setSensors(INV_XYZ_ACCEL | INV_XYZ_GYRO);

  /* Configure sensors: */
  imu.setGyroFSR( IMU_GYRO_FSR );
  imu.setAccelFSR( IMU_ACCEL_FSR );
  imu.setLPF( IMU_AG_LPF );
  imu.setSampleRate( IMU_AG_SAMPLE_RATE );
  return true; // Return success
}



