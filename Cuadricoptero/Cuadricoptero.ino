#ifndef QUADARDU
#define QUADARDU

#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
float kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
//#include <PinChangeInt.h>
//#include <PinChangeIntConfig.h>

///Librerias BT PS4
//#include <PS4BT.h>
//#include <usbhub.h>
//
////Satisfy the IDE, which needs to see the include statment in the ino too.
//#ifdef dobogusinclude
//#include <spi4teensy3.h>
//#include <SPI.h>
//#endif

//USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside
//BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so

/* You can create the instance of the PS4BT class in two ways */
// This will start an inquiry and then pair with the PS4 controller - you only have to do this once
// You will need to hold down the PS and Share button at the same time, the PS4 controller will then start to blink rapidly indicating that it is in paring mode
//PS4BT PS4(&Btd, PAIR);

// After that you can simply create the instance like so and then press the PS button on the device
//PS4BT PS4(&Btd);


#define DEBUG


/*  Arduino Pin configuration
 *
 */

#define ESC_B 7
#define ESC_D 6
#define ESC_A 5
#define ESC_C 4

//#define RC_1 13
//#define RC_2 12
//#define RC_3 11
//#define RC_4 10
//#define RC_5 8
//#define RC_PWR A0


/* ESC configuration
 *
 */

#define ESC_MIN 600
#define ESC_MAX 2000
#define ESC_TAKEOFF_OFFSET 600
#define ESC_ARM_DELAY 5000

/* RC configuration
 *
 */

#define RC_HIGH_CH1 0
#define RC_LOW_CH1 255
#define RC_HIGH_CH2 0
#define RC_LOW_CH2 255
#define RC_HIGH_CH3 0
#define RC_LOW_CH3 255
#define RC_HIGH_CH4 0
#define RC_LOW_CH4 255
#define RC_HIGH_CH5 0
#define RC_LOW_CH5 255
#define RC_ROUNDING_BASE 50

/*  PID configuration
 *
 */

#define PITCH_P_VAL 0.5
#define PITCH_I_VAL 0
#define PITCH_D_VAL 1

#define ROLL_P_VAL 5
#define ROLL_I_VAL 2
#define ROLL_D_VAL 2

#define YAW_P_VAL 2
#define YAW_I_VAL 5
#define YAW_D_VAL 1


/* Flight parameters
 *
 */

#define PITCH_MIN 0
#define PITCH_MAX 0
#define ROLL_MIN -0.1
#define ROLL_MAX 0.1
#define YAW_MIN -180
#define YAW_MAX 180
#define PID_PITCH_INFLUENCE 20
#define PID_ROLL_INFLUENCE 150
#define PID_YAW_INFLUENCE 20


/*  MPU variables
 *
 */

MPU6050 mpu;                           // mpu interface object


uint8_t mpuIntStatus;                  // mpu statusbyte
uint8_t devStatus;                     // device status
uint16_t packetSize;                   // estimated packet size
uint16_t fifoCount;                    // fifo buffer size
uint8_t fifoBuffer[64];                // fifo buffer

Quaternion q;                          // quaternion for mpu output
VectorFloat gravity;                   // gravity vector for ypr
float ypr[3] = {0.0f, 0.0f, 0.0f};     // yaw pitch roll values
float yprLast[3] = {0.0f, 0.0f, 0.0f};

volatile bool mpuInterrupt = false;    //interrupt flag

/* Interrupt lock
 *
 */

//boolean interruptLock = false;

/*  RC variables
 *
 */

float ch1, ch2, ch3, ch4, ch5;         // RC channel inputs

//unsigned long rcLastChange1 = micros();
//unsigned long rcLastChange2 = micros();
//unsigned long rcLastChange3 = micros();
//unsigned long rcLastChange4 = micros();
//unsigned long rcLastChange5 = micros();

/*  Motor controll variables
 *
 */

float velocity;                          // global velocity

float bal_ac, bal_bd;                 // motor balances can vary between -100 & 100
float bal_axes;                       // throttle balance between axes -100:ac , +100:bd

float va, vb, vc, vd;                    //velocities
float v_ac, v_bd;                        // velocity of axes

Servo a, b, c, d;

/*  PID variables
 *
 */

PID pitchReg(&ypr[2], &bal_bd, &ch2, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, REVERSE);
PID rollReg(&kalAngleX, &bal_ac, &ch1, ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL, REVERSE);
PID yawReg(&ypr[0], &bal_axes, &ch4, YAW_P_VAL, YAW_I_VAL, YAW_D_VAL, DIRECT);



/*  Filter variables
 *
 */

float ch1Last, ch2Last, ch4Last, velocityLast;

/*  Setup function
 *
 */

void setup() {

  //initRC();                            // Self explaining
  //initMPU();
  initESCs();
  initBalancing();
  initRegulators();

#ifdef DEBUG                        // Device tests go here

  Serial.begin(115200);                 // Serial only necessary if in DEBUG mode
  Serial.flush();

#endif
}

/* loop function
 *
 */

void loop() {

  //  while(!mpuInterrupt && fifoCount < packetSize){
  //
  //    /* Do nothing while MPU is not working
  //     * This should be a VERY short period
  //     */
  //
  // }
  //getYPR();
  KalmanCorrection();
  computePID();
  calculateVelocities();
  updateMotors();

}


#endif




