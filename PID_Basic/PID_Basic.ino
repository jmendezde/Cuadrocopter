/********************************************************
 * PID Basic Example
 * Reading analog InputRoll 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>
#include <Servo.h>
#define PIN_InputRoll 0
#define PIN_OUTPUT 3
///Sensor IMU
#include "Wire.h"
#include "I2Cdev.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "ITG3200.h"

#define PITCH_MIN -30
#define PITCH_MAX 30
#define ROLL_MIN -50
#define ROLL_MAX 50
#define ESC_MIN 750
#define ESC_MAX 1900
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
#define YAW_MIN -180
#define YAW_MAX 180
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

double ch1, ch2, ch3, ch4;
//Define Variables we'll be connecting to
double SetpointRoll, SetpointPitch, SetpointYaw, InputRoll, InputPitch, InputYaw, Output, Output1, Output2;
double ypr[3] = {0, 0, 0};
double yprLast[3] = {0, 0, 0};
//Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
PID rollReg(&InputRoll, &Output, &SetpointRoll, Kp, Ki, Kd, REVERSE);
PID pitchReg(&InputPitch, &Output1, &SetpointPitch, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, REVERSE);
PID yawReg(&InputYaw, &Output2, &SetpointYaw, Kp, Ki, Kd, DIRECT);




void setup()
{
  Serial.begin(115200);//comunicacion serial
  intARM();
  delay(ESC_ARM_DELAY);
  initBalancing();
  initRegulators();
  initIMU();
  //initialize the variables we're linked to
  InputRoll = ypr[0];
  InputPitch = ypr[1];
  InputYaw = ypr[2];
  SetpointRoll = 30;
  SetpointPitch = 10;
  SetpointYaw = 107;
}
void loop()
{

  IMU();
  computePID();
  calculateVelocities();
  updateMotors();
  //delay(100);
  Serial.print("\tSalida:A\t");
  Serial.print(va);
  Serial.print("\tSalida:B\t");
  Serial.print(vb);
  Serial.print("\tSalida:C\t");
  Serial.print(vc);
  Serial.print("\tSalida:D\t");
  Serial.print(vd);
  Serial.print("\r\n");
}











