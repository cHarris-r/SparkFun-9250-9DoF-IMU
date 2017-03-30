



/*************************************************
** f_InitHardware 
** This function sets the LED GPIO 
*/
void f_InitHardware( void )
{
	/* Some Log Output (usb) */
	LOG_PORT.println("> Initializing Hardware");
		
	/* Set up LED pin (active-high, default to off) */
	pinMode(HW_LED_PIN, OUTPUT);
	digitalWrite(HW_LED_PIN, LOW);

	/* Set up MPU-9250 interrupt input (active-low) */
	pinMode(MPU9250_INT_PIN, INPUT_PULLUP);
}


/*************************************************
**
**
*/
bool initIMU(void)
{
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




/*************************************************
** f_BLinkLED 
** This function is used to communicate to the user
** that the board is indeed doing things
** TO DO: I plan to implement a blink code for debugging 
*/
void f_BlinkLED( void )
{
  /* We blink every UART_BLINK_RATE millisecods */
  if ( millis() > (g_LastBlinkTime + UART_BLINK_RATE) )
  {
    Debug_LogOut();
    
    LOG_PORT.println("> Blink ...");
    LOG_PORT.println("> # Available on COMM_PORT: " + String(COMM_PORT.available()) );
    digitalWrite(HW_LED_PIN, g_LedState);
    g_LedState = !g_LedState;
    g_LastBlinkTime = millis();
  }
} /* End f_BLinkLED */




