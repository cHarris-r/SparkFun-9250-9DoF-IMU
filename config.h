


/* IMU Default Configs */
#define SRATE 200
#define IMU_AG_SAMPLE_RATE 10000 // Accel/gyro sample rate Must be between 4Hz and 1kHz
#define IMU_GYRO_FSR       2000 // Gyro full-scale range (250, 500, 1000, or 2000)
#define IMU_ACCEL_FSR      2 // Accel full-scale range (2, 4, 8, or 16)
#define IMU_AG_LPF         5 // Accel/Gyro LPF corner frequency (5, 10, 20, 42, 98, or 188 Hz)

/* DCM parameters */
#define Kp_ROLLPITCH 0.1f//0.02f
#define Ki_ROLLPITCH 0.00005//0.00002f

#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

#define GRAVITY 256.0f//285.0f // "1G reference" used for DCM filter and accelerometer calibration

/* MACROS */
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

#define ACCEL_GAIN 0.0134//0.0151
#define ACCEL_SCALED(x) (x * ACCEL_GAIN) 


/* Serial Port Config */
#define DEBUG 1
//#define LOG_PORT if(DEBUG)Serial
#define LOG_PORT if(DEBUG)SERIAL_PORT_USBVIRTUAL

#define COMM_PORT Serial1


/* LED Config */
#define UART_BLINK_RATE 1000
#define HW_LED_PIN      13 

/* Hardware Definitions / */
// Danger - don't change unless using a different platform
#define MPU9250_INT_PIN 4
#define MPU9250_INT_ACTIVE LOW



/*******************************************************************
** Tyedefs *********************************************************
********************************************************************/
/* The RESPONSE_TYPE is used 
** to store temporary resonse data
** for responding to request from master */
typedef struct{
  uint16_t Packet_nBytes;  /* Length of entire packet, minus this variable, in bytes */
  uint16_t PacketType;     /* Type code of packet */
  uint16_t Buffer_nBytes;  /* Length of data buffer in bytes (0-50) */
  unsigned char  Buffer[50];     /* Data buffer */
  unsigned char  CheckSum;       /* CheckSum of data buffer only */
} RESPONSE_TYPE;




