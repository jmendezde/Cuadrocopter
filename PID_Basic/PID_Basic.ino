/********************************************************
 * PID Basic Example
 * Reading analog InputRoll 0 to control analog PWM output 3
 ********************************************************/
//
//#ifdef dobogusinclude
//#include <spi4teensy3.h>
//#include <SPI.h>
//#endif
//#include <PS4BT.h>
//#include <usbhub.h>
#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
#define gyroAddress 0x68
#define adxlAddress 0x53


double zeroValue[5] = { 10, 0, 227, 679, 28}; // Found by experimenting

/* All the angles start at 180 degrees */
double gyroXangle = 180;
double gyroYangle = 180;

double compAngleX = 180;
double compAngleY = 180;

double xAngle = 0;
double yAngle = 0;

unsigned long timer;

uint8_t buffer[2]; // I2C buffer
///Sensor IMU
//#include "Wire.h"
#include "I2Cdev.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "ITG3200.h"

#define PITCH_MIN 0
#define PITCH_MAX 90
#define ROLL_MIN 0
#define ROLL_MAX 90
#define YAW_MIN 180
#define YAW_MAX 180

#define ESC_MIN 600
#define ESC_MAX 1900
#define ESC_TAKEOFF_OFFSET 30
#define ESC_ARM_DELAY 10
/* Interrupt lock
 *
 */


float ch1Last, ch2Last, ch4Last, velocityLast;
int velocity;                          // global velocity

int va, vb, vc, vd;                    //velocities
int v_cd, v_ab,v_cb,v_ad;                        // velocity of axes
Servo cuartoESC, tercerESC, primerESC, segundoESC, ServoCam; //declaro los servos
#define ESC_A 7
#define ESC_B 6
#define ESC_C 5
#define ESC_D 4
/* RC configuration
 *
 */

#define RC_HIGH_CH1 255
#define RC_LOW_CH1 0
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

int16_t tempRaw;

#define PID_PITCH_INFLUENCE 255
#define PID_ROLL_INFLUENCE 255
#define PID_YAW_INFLUENCE 255

double ch1, ch2, ch3, ch4;
//Define Variables we'll be connecting to
double SetpointRoll, SetpointPitch, SetpointYaw, InputRoll, InputPitch, InputYaw, Output, Output1, Output2;
double ypr[3] = {0, 0, 0};
double yprLast[3] = {0, 0, 0};
//Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
PID rollReg(&ypr[0], &Output, &SetpointRoll, Kp, Ki, Kd, REVERSE);
PID pitchReg(&ypr[1], &Output1, &SetpointPitch, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, REVERSE);
PID yawReg(&ypr[2], &Output2, &SetpointYaw, Kp, Ki, Kd, DIRECT);

///* Configuracion de Mando PS4
// *
// */
//
//USB Usb;//Declaro la conexion BT con el Mando PS4
//BTD Btd(&Usb);
////PS4BT PS4(&Btd, PAIR);
//PS4BT PS4(&Btd);



void setup()
{
  intARM();
  // delay(ESC_ARM_DELAY);------------------------------IMPORTANTE-------------------------------------
  Serial.begin(115200);

  initKalman();
  initBalancing();
  initRegulators();
  initIMU();
  initSetpoints();

}
void loop()
{

  //MANDOPS4();//Subrutina Configuracion del mando de PS4
  IMU();
  //KalmanCorrection();
  computePID();
  calculateVelocities();
  updateMotors();
  delay(100);
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











