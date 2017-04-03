#include <SparkFunMPU9250-DMP.h> 
#include <string.h>
#include "config.h"
#include <Wire.h>
#include <Arduino.h>


/*******************************************************************
** Globals *********************************************************
********************************************************************/

/* Serial communication globals */
//static bool g_BaudLock = false; /* Used to set baud rate */
static bool g_BaudLock = true; /* Used to set baud rate */

/* LED state globals */
static bool g_LedState = false; /* Used to set LED state */
uint32_t    g_LastBlinkTime = 0;   /* Used to set LED state */

/* DCM variables */
MPU9250_DMP imu; 
float g_MAG_Heading;
float Accel_Vector[3]        = {0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3]         = {0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3]        = {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3]             = {0, 0, 0}; // Omega Proportional correction
float Omega_I[3]             = {0, 0, 0}; // Omega Integrator
float Omega[3]               = {0, 0, 0};
float errorRollPitch[3]      = {0, 0, 0};
float errorYaw[3]            = {0, 0, 0};
float DCM_Matrix[3][3]       = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3]    = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

float yaw   = -1.0;
float pitch =  2.5;
float roll  = -3.25;
float mag[]   = {0,0,0};
float accel[] = {0,0,0};
float gyro[]  = {0,0,0};

/* DCM timing in the main loop */
unsigned long timestamp     = 0;
unsigned long timestamp_old = 0;
float G_Dt = 0; // Integration time for DCM algorithm
float mydt = 0;
int count  = 0;




/*******************************************************************
** START ***********************************************************
********************************************************************/

/*************************************************
** Setup Function 
** This function contains the setup functions 
** including the initialization of the hardware
** and the initialization of the serial ports
*/
void setup() {
	/* Initialize Baud rate 
	** NOTE: The master will adaptivly determine baud
	**       therefore, changing the baud rate here should 
	**       not change the master code 
	** !!!!  This feature is not active !!!! */

	/* Initialize the LED GPIO */
	Init_Hardware();

  /* Initialize the MPU-9250. Should return true on success: */
  if ( !Init_IMU() ) 
  {
    LOG_PORT.println("Error connecting to MPU-9250");
    while (1) ; // Loop forever if we fail to connect
  }
  LOG_PORT.println("> IMU Initialized");
  delay(20);

  /* Set the initial roll/pitch/yaw from 
  ** initial accel/gyro */
  Reset_Sensor_Fusion(); 
}


/*************************************************
** Main Start
** This loop is essentially the main function call
** for the code.
*/
void loop() 
{
	int i;
  float test;
	uint8_t test2 = 0x01;
 
	/* Data IO Varaibles */
	uint32_t nBytesIn;
	uint16_t ByteIn;
	RESPONSE_TYPE Response;

	/* We need to have Baud Lock to communicate
	** The master code (the TI board) uses Auto Baud
	** To Boud Lock, we must send the character "a" or "A"
	** when requested 
	** THIS IS DIACTIVATED FOR NOW */
	if( g_BaudLock == false ) { f_Handshake(); }

	/* If Baud is locked we can properly 
	** communicate data with the master
	** The master will send a request character, 
	** which will tell us what data to send. 
	** We respond with the proper data packet */
	else 
	{
    nBytesIn = COMM_PORT.available();
    if( nBytesIn>0 ){ f_SendData( nBytesIn ); }
	}

	/* Oproational code */
  imu.updateAccel();
  accel[0] = imu.ax;
  accel[1] = imu.ay;
  accel[2] = imu.az;
  imu.updateGyro();
  gyro[0] = imu.gx;
  gyro[1] = imu.gy;
  gyro[2] = imu.gz;

  Read_Sensors();
	Update_Time();
  Matrix_Update();
  Normalize();
  Drift_Correction();
  Euler_Angles();
  
  
	/* Blink LED 
	** TO DO: It would be nice to have a blink code
	**        to communicate during operation */
	f_BlinkLED();
} /* End Main Loop */





































