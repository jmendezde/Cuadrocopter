/********************************************************
 * PID Basic Example
 * Reading analog InputRoll 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>
#include <Servo.h>

#include <Wire.h>
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
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine
///Sensor IMU
//#include "Wire.h"
#include "I2Cdev.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "ITG3200.h"

#define PITCH_MIN -30
#define PITCH_MAX 30
#define ROLL_MIN -50
#define ROLL_MAX 50
#define YAW_MIN -180
#define YAW_MAX 180

#define ESC_MIN 750
#define ESC_MAX 2000
#define ESC_TAKEOFF_OFFSET 30
#define ESC_ARM_DELAY 10
/* Interrupt lock
 *
 */


float ch1Last, ch2Last, ch4Last, velocityLast;
int velocity;                          // global velocity

int va, vb, vc, vd;                    //velocities
int v_ac, v_bd;                        // velocity of axes
Servo cuartoESC, tercerESC, primerESC, segundoESC, ServoCam; //declaro los servos
#define ESC_A 7
#define ESC_B 6
#define ESC_C 5
#define ESC_D 4
/* RC configuration
 *
 */

#define RC_HIGH_CH1 0
#define RC_LOW_CH1 255
#define RC_ROUNDING_BASE 50

#define PITCH_P_VAL 0.5
#define PITCH_I_VAL 0
#define PITCH_D_VAL 1

//Declaro los sensores del IMU
// class default I2C address is 0x53
// specific I2C addresses may be passed as a parameter here
// ALT low = 0x53 (default for SparkFun 6DOF board)
// ALT high = 0x1D
HMC5883L mag;
ITG3200 gyro;
ADXL345 accel;

int16_t ax, ay, az;
int16_t mx, my, mz;
int16_t gx, gy, gz;


#define PID_PITCH_INFLUENCE 20
#define PID_ROLL_INFLUENCE 20
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
bool mpuInterrupt = false;    //interrupt flag


double ch1, ch2, ch3, ch4;
//Define Variables we'll be connecting to
float SetpointRoll, SetpointPitch, SetpointYaw, InputRoll, InputPitch, InputYaw, Output, Output1, Output2;

//Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;

PID rollReg(&ypr[0], &Output, &SetpointRoll, Kp, Ki, Kd, REVERSE);
PID pitchReg(&ypr[1], &Output1, &SetpointPitch, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, REVERSE);
PID yawReg(&ypr[2], &Output2, &SetpointYaw, Kp, Ki, Kd, DIRECT);

void setup()
{
  intARM();
  delay(ESC_ARM_DELAY);
  Serial.begin(115200);
  Wire.begin();
  initKalman();
  initBalancing();
  initRegulators();
  initIMU();
  initSetpoints();

}
void loop()
{
  IMU();
  KalmanCorrection();
  computePID();
  calculateVelocities();
  updateMotors();
  //delay(100);
  Serial.print("\tVA\t");
  Serial.print(va);
  Serial.print("\tVB\t");
  Serial.print(vb);
  Serial.print("\tVC\t");
  Serial.print(vc);
  Serial.print("\tVD\t");
  Serial.print(vd);
  Serial.print("\r\n");
}











