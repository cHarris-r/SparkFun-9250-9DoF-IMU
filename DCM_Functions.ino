
/*************************************************
** Init_Rotation_Matrix
** Init rotation matrix using euler angles 
*/
void Init_Rotation_Matrix(float m[3][3], float yaw, float pitch, float roll)
{
  float c1 = cos(roll);
  float s1 = sin(roll);
  float c2 = cos(pitch);
  float s2 = sin(pitch);
  float c3 = cos(yaw);
  float s3 = sin(yaw);

  // Euler angles, right-handed, intrinsic, XYZ convention
  // (which means: rotate around body axes Z, Y', X'') 
  m[0][0] = c2 * c3;
  m[0][1] = c3 * s1 * s2 - c1 * s3;
  m[0][2] = s1 * s3 + c1 * c3 * s2;

  m[1][0] = c2 * s3;
  m[1][1] = c1 * c3 + s1 * s2 * s3;
  m[1][2] = c1 * s2 * s3 - c3 * s1;

  m[2][0] = -s2;
  m[2][1] = c2 * s1;
  m[2][2] = c1 * c2;
}


/*************************************************
** Compass_Heading
** Get the current compas heading wrt north
** Uses estimated mag data
** This function is not being used!
*/
void Compass_Heading()
{
  float mag_x;
  float mag_y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll  = cos(roll);
  sin_roll  = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  
	/* Tilt compensated magnetic field X */
  mag_x = mag[0] * cos_pitch + mag[1] * sin_roll * sin_pitch + mag[2] * cos_roll * sin_pitch;
  
	/* Tilt compensated magnetic field Y */
  mag_y = mag[1] * cos_roll - mag[2] * sin_roll;
  
	/* Magnetic Heading */
  MAG_Heading = f_atan2(-mag_y, mag_x);
}


/*************************************************
** f_UpdateTime
** 
*/
void f_UpdateTime( void )
{
  float temp; 
  
  timestamp_old = timestamp;
  timestamp     = micros();
  temp = timestamp - timestamp_old;

  
  //while ( 1000000/(timestamp - timestamp_old) > (SRATE) ) { timestamp = micros(); }
  
  if( timestamp_old > 0 ) { G_Dt = (temp / 1000000.0) ; }
  else { G_Dt = 0; }
  
  mydt += (timestamp - timestamp_old);
}


/*************************************************
** Matrix_Update
**
*/
void Matrix_Update( void )
{
  Gyro_Vector[0] = GYRO_SCALED_RAD( gyro[0] ); //gyro x roll
  Gyro_Vector[1] = GYRO_SCALED_RAD( gyro[1] ); //gyro y pitch
  Gyro_Vector[2] = GYRO_SCALED_RAD( gyro[2] ); //gyro z yaw
  
  Accel_Vector[0] = ACCEL_SCALED( accel[0] );
  Accel_Vector[1] = ACCEL_SCALED( accel[1] );
  Accel_Vector[2] = ACCEL_SCALED( accel[2] );

  Omega_Vector[0] = Gyro_Vector[0] + Omega_I[0] + Omega_P[0];
  Omega_Vector[1] = Gyro_Vector[1] + Omega_I[1] + Omega_P[1];
  Omega_Vector[2] = Gyro_Vector[2] + Omega_I[2] + Omega_P[2];

  DCM_Matrix[0][0] = G_Dt * (DCM_Matrix[0][1]*Omega_Vector[2] - DCM_Matrix[0][2]*Omega_Vector[1]) + DCM_Matrix[0][0];
  DCM_Matrix[0][1] = G_Dt * (DCM_Matrix[0][2]*Omega_Vector[0] - DCM_Matrix[0][0]*Omega_Vector[2]) + DCM_Matrix[0][1];
  DCM_Matrix[0][2] = G_Dt * (DCM_Matrix[0][0]*Omega_Vector[1] - DCM_Matrix[0][1]*Omega_Vector[0]) + DCM_Matrix[0][2];
  
  DCM_Matrix[1][0] = G_Dt * (DCM_Matrix[1][1]*Omega_Vector[2] - DCM_Matrix[1][2]*Omega_Vector[1]) + DCM_Matrix[1][0];
  DCM_Matrix[1][1] = G_Dt * (DCM_Matrix[1][2]*Omega_Vector[0] - DCM_Matrix[1][0]*Omega_Vector[2]) + DCM_Matrix[1][1];
  DCM_Matrix[1][2] = G_Dt * (DCM_Matrix[1][0]*Omega_Vector[1] - DCM_Matrix[1][1]*Omega_Vector[0]) + DCM_Matrix[1][2];
}


/*************************************************
** Normalize
**
*/
void Normalize(void)
{
  float error=0;
  float temporary[3][3];
  float renorm=0;

  error = -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0]) * 0.5; 
  
  Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); 
  Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); 
  
  Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);
  Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);
   
  Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); 
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0]));
  Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0]));
  Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); 
  Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}


/*************************************************
** Drift_Correction
**
*/
void Drift_Correction(void)
{
  //Compensation the Roll, Pitch drift. 
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  
  float Accel_magnitude;
  float Accel_weight;
  
  float mag_heading_x;
  float mag_heading_y;
  float errorCourse;
  
  /* **** Roll and Pitch *************** */
  /* Calculate the magnitude of the accelerometer vector */
  Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  
  /* Dynamic weighting of accelerometer info (reliability filter)
	** Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0) */
  Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1); 

  /* Adjust the ground of reference */
  Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); 
  Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);
  
  Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);     

  /* ***** YAW *************** */
  /* We make the gyro YAW drift correction based on compass magnetic heading */
 
  mag_heading_x = 1;
  mag_heading_y = 0;
  errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  /* Calculating YAW error */
  Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); /* Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position. */
  
  Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW); /* 01proportional of YAW. */
  Vector_Add(Omega_P,Omega_P,Scaled_Omega_P); /* Adding  Proportional. */
  
  Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW); /* Adding Integrator */
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
}


/*************************************************
** Euler_Angles
**
*/
void Euler_Angles(void)
{
  pitch = -f_asin( DCM_Matrix[2][0] ); // A faster asin
  roll  =  f_atan2( DCM_Matrix[2][1], DCM_Matrix[2][2] ); // A faster atan2
  yaw   =  f_atan2( DCM_Matrix[1][0], DCM_Matrix[0][0] ); // A faster atan2
}





/*************************************************
** Reset_Sensor_Fusion
** Read every sensor and record a time stamp
** Init DCM with unfiltered orientation
*/
void Reset_Sensor_Fusion() {
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  /* GET PITCH
  ** Using y-z-plane-component/x-component of gravity vector */
  pitch = -f_atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));

  /* GET ROLL
  ** Compensate pitch of gravity vector */
  Vector_Cross_Product(temp1, accel, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  
	/* Normally using x-z-plane-component/y-component of compensated gravity vector
  ** roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  ** Since we compensated for pitch, x-z-plane-component equals z-component: */
  roll = f_atan2(temp2[1], temp2[2]);

  /* GET YAW */
  yaw = 0;

  /* Init rotation matrix */
  Init_Rotation_Matrix(DCM_Matrix, yaw, pitch, roll);
}


/*************************************************
** Set_Sensor_Fusion
**
*/
void Set_Sensor_Fusion()
{
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  /* GET PITCH
  ** Using y-z-plane-component/x-component of gravity vector */
  pitch = -f_atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));

  /* GET ROLL
	** Compensate pitch of gravity vector */
  Vector_Cross_Product(temp1, accel, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  
	/* Normally using x-z-plane-component/y-component of compensated gravity vector
	** roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
	** Since we compensated for pitch, x-z-plane-component equals z-component: */
  roll = f_atan2(temp2[1], temp2[2]);

  /* GET YAW */
  yaw = 0;
}


