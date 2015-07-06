#include <Wire.h>
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
//#include "Config.h"
#include <Servo.h>

#define PID_Pitch_Kp 0.07
#define PID_Pitch_Ki 0.00001
#define PID_Pitch_Kd 0.05

#define PID_Roll_Kp 0.07
#define PID_Roll_Ki 0.00001
#define PID_Roll_Kd 0.05

#define PID_Pitch_MaxValue 15
#define PID_Pitch_MinValue -15
#define PID_Roll_MaxValue 15
#define PID_Roll_MinValue -15

#define Max_Pitch_Angle 15
#define Min_Pitch_Angle -15
#define Max_Roll_Angle 15
#define Min_Roll_Angle -15

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN1 9
#define MOTOR_PIN2 10
#define MOTOR_PIN3 11
#define MOTOR_PIN4 12

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;
MPU6050 mpu;
byte incomingByte = 0;
double PitchAngle = 0, RollAngle = 0;
double Desired_Pitch_Angle = 0.000, Desired_Roll_Angle = 0.000;
double Desired_Throttle = 0.00;
double Roll_P_coef = 0.000, Roll_I_coef = 0.000, Roll_D_coef = 0.000;
//double Pitch_P_coef=0.00,Pitch_I_coef=0.00,Pitch_D_coef=0.00;
bool dmpReady = false;
double PID_Pitch_value = 0.000, PID_Roll_value = 0.000;
uint8_t mpuIntStatus;
uint16_t packetSize;
unsigned long  tpPitch = millis();
unsigned long  tpRoll = millis();
double pErrorPitch = 0.0, pErrorRoll = 0.0;
double IpPitch = 0.0, IpRoll = 0.0;
double motor_value1 = 0.00, motor_value2 = 0.00, motor_value3 = 0.00, motor_value4 = 0.00;
bool IsArm = true;
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  TWBR = 24;
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setXAccelOffset(-1512);
  mpu.setYAccelOffset(2159);
  mpu.setZAccelOffset(1329);
  mpu.setXGyroOffset(27);
  mpu.setYGyroOffset(-22);
  mpu.setZGyroOffset(347);
  mpu.setDMPEnabled(true);
  attachInterrupt(0, dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();
  motor1.attach(MOTOR_PIN1, MIN_SIGNAL, MAX_SIGNAL);
  motor2.attach(MOTOR_PIN2, MIN_SIGNAL, MAX_SIGNAL);
  motor3.attach(MOTOR_PIN3, MIN_SIGNAL, MAX_SIGNAL);
  motor4.attach(MOTOR_PIN4, MIN_SIGNAL, MAX_SIGNAL);
  motor1.write(0);
  motor2.write(0);
  motor3.write(0);
  motor4.write(0);
}
void loop()
{
  GetThrottleAndDesiredAngles();
  Get_angles_from_sensor();
  if (IsArm == true)
  {
    PID_Pitch_value = PIDCompute(Desired_Pitch_Angle - PitchAngle, PID_Pitch_Kp, PID_Pitch_Ki, PID_Pitch_Kd, PID_Pitch_MaxValue, PID_Pitch_MinValue, tpPitch, pErrorPitch, IpPitch);
    PID_Roll_value = PIDCompute(Desired_Roll_Angle - RollAngle, Roll_P_coef, Roll_I_coef, Roll_D_coef, PID_Roll_MaxValue, PID_Roll_MinValue, tpRoll, pErrorRoll, IpRoll);
    motor_value1 = mapdouble(Desired_Throttle + PID_Pitch_value, 0, 100, MIN_SIGNAL, MAX_SIGNAL);
    motor_value4 = mapdouble(Desired_Throttle - PID_Roll_value, 0, 100, MIN_SIGNAL, MAX_SIGNAL);
    motor_value3 = mapdouble(Desired_Throttle - PID_Pitch_value, 0, 100, MIN_SIGNAL, MAX_SIGNAL);
    motor_value2 = mapdouble(Desired_Throttle + PID_Roll_value, 0, 100, MIN_SIGNAL, MAX_SIGNAL);
    //    Serial.print(motor_value1);
    //    Serial.print("\t");
    //    Serial.print(motor_value2);
    //    Serial.print("\t");
    //    Serial.print(motor_value3);
    //    Serial.print("\t");
    //    Serial.print(motor_value4);

    if (motor_value4 < 1000)
    {
      motor_value4 = 1000;
    }
    if (motor_value2 < 1000)
    {
      motor_value2 = 1000;
    }
    if (motor_value4 > 2000)
    {
      motor_value4 = 2000;
    }
    if (motor_value2 > 2000)
    {
      motor_value2 = 2000;
    }
    motor1.writeMicroseconds(motor_value1);
    motor2.writeMicroseconds(motor_value2);
    motor3.writeMicroseconds(motor_value3);
    motor4.writeMicroseconds(motor_value4);
  }
  Serial.print(Desired_Pitch_Angle);
  Serial.print(",");
  Serial.print(PitchAngle);
  Serial.print(",");
  Serial.print(PID_Pitch_value);
  Serial.print(",");
  Serial.print(Desired_Roll_Angle);
  Serial.print(",");
  Serial.print(RollAngle);
  Serial.print(",");
  Serial.print(PID_Roll_value);
  Serial.print(",");
  Serial.print(Desired_Throttle);
  Serial.print("\tM1\t");
  Serial.print(motor_value1);
  Serial.print("\tM2\t");
  Serial.print(motor_value2);
  Serial.print("\tM3\t");
  Serial.print(motor_value3);
  Serial.print("\tM4\t");
  Serial.print(motor_value4);
  Serial.print(",");
  Serial.print(Roll_P_coef, 3);
  Serial.print(",");
  Serial.print(Roll_I_coef, 4);
  Serial.print(",");
  Serial.println(Roll_D_coef, 3);
}
double mapdouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
uint8_t devStatus;
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity;
volatile bool mpuInterrupt = false;
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];
#include "I2Cdev.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "ITG3200.h"

HMC5883L mag;
ITG3200 gyro;
ADXL345 accel;

int16_t ax, ay, az;
int16_t mx, my, mz;
int16_t gx, gy, gz;
float ypr[3];
void dmpDataReady()
{
  Wire.begin();
  accel.initialize();
  mag.initialize();
  gyro.initialize();
  ypr[0] = 0;
  ypr[1] = 0;
  ypr[2] = 0;
}
void Get_angles_from_sensor()
{
  accel.getAcceleration(&ax, &ay, &az);
  mag.getHeading(&mx, &my, &mz);
  gyro.getRotation(&gx, &gy, &gz);
  ypr[0] = ax;
  ypr[1] = ay;
  //  ypr[2] = my;
  RollAngle = ypr[0] * 180 / M_PI;
  PitchAngle = ypr[1] * 180 / M_PI;
}
void GetThrottleAndDesiredAngles()
{
  if (Serial.available() > 0)
  {
    incomingByte = Serial.read();
    switch (incomingByte)
    {
      case 81:
        throttle_checking(-1);
        break;
      case 69:
        throttle_checking(1);
        break;
      case 87: //numpad8
        Desired_Pitch_Angle = angle_checking(Desired_Pitch_Angle, 1, Max_Pitch_Angle, Min_Pitch_Angle);
        break;
      case 83: //numpad2
        Desired_Pitch_Angle = angle_checking(Desired_Pitch_Angle, -1, Max_Pitch_Angle, Min_Pitch_Angle);
        break;
      case 65: //numpad4
        Desired_Roll_Angle = angle_checking(Desired_Roll_Angle, -1, Max_Roll_Angle, Min_Roll_Angle);
        break;
      case 68: //numpad6
        Desired_Roll_Angle = angle_checking(Desired_Roll_Angle, 1, Max_Roll_Angle, Min_Roll_Angle);
        break;
      case 90: //numpad6
        IsArm = false;
        motor1.write(0);
        motor2.write(0);
        motor3.write(0);
        motor4.write(0);
        break;
      case 85:
        Roll_P_coef = angle_checking(Roll_P_coef, 0.001, Max_Roll_Angle, Min_Roll_Angle);
        break;
      case 74:
        Roll_P_coef = angle_checking(Roll_P_coef, -0.001, Max_Roll_Angle, Min_Roll_Angle);
        break;
      case 73:
        Roll_I_coef = angle_checking(Roll_I_coef, 0.0001, Max_Roll_Angle, Min_Roll_Angle);
        break;
      case 75:
        Roll_I_coef = angle_checking(Roll_I_coef, -0.0001, Max_Roll_Angle, Min_Roll_Angle);
        break;
      case 79:
        Roll_D_coef = angle_checking(Roll_D_coef, 0.001, Max_Roll_Angle, Min_Roll_Angle);
        break;
      case 76:
        Roll_D_coef = angle_checking(Roll_D_coef, -0.001, Max_Roll_Angle, Min_Roll_Angle);
        break;
    }
  }
}
void throttle_checking(short number)
{
  if (Desired_Throttle + number <= 100 && Desired_Throttle + number >= 0)
  {
    Desired_Throttle += number;
  }
}

double angle_checking(double angle, double number, double maxval, double minval)
{
  if (angle + number <= maxval && angle + number >= minval)
  {
    angle += number;
  }
  return angle;
}

double PIDCompute(double mError, double kp, double ki, double kd, double Hval, double Lval, unsigned long  tp, double pError, double Ip)
{
  unsigned long tn = millis();
  double dt = (double)(tn - tp);
  double P = (double)kp * mError;
  double D = (double)(kd * (mError - pError) * 1000.0 / dt);
  pError = mError;
  double I = (double)(Ip + ki * mError * dt / 1000.0);
  double U = (double)(P + I + D);
  Ip = I;
  tp = tn;
  if (U > Hval)
  {
    U = Hval;
  }
  else if (U < Lval)
  {
    U = Lval;
  }
  return U;
}
