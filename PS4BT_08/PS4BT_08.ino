#include <PS4BT.h>
#include <usbhub.h>
#include <Servo.h>
#include <helper_3dmath.h>
#include <I2Cdev.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif
#include <PID_v1.h>
///Sensor IMU
#include "Wire.h"
#include "I2Cdev.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "ITG3200.h"

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

/* Configuracion de ESC's
 *
 */
Servo cuartoESC, tercerESC, primerESC, segundoESC, ServoCam; //declaro los servos
#define ESC_A 7
#define ESC_B 6
#define ESC_C 5
#define ESC_D 4
//#define SERVOCAM 8
//int pos = 0;    // variable to store the servocam position
#define ESC_MIN 600
#define ESC_MAX 2000
#define ESC_TAKEOFF_OFFSET 30
#define ESC_ARM_DELAY 10000

float ch1, ch2, ch3, ch4;         // PS4 mando inputs
unsigned long UltimoCambio1 = micros();
unsigned long UltimoCambio2 = micros();
unsigned long UltimoCambio3 = micros();
unsigned long UltimoCambio4 = micros();
/* RC configuration
 *
 */

#define RC_HIGH_CH1 1000
#define RC_LOW_CH1 2000
#define RC_ROUNDING_BASE 50

/* Flight parameters
 *
 */

#define PITCH_MIN -30
#define PITCH_MAX 30
#define ROLL_MIN -30
#define ROLL_MAX 30
#define YAW_MIN -180
#define YAW_MAX 180
#define PID_PITCH_INFLUENCE 20
#define PID_ROLL_INFLUENCE 20
#define PID_YAW_INFLUENCE 20
/*  PID configuration
 *
 */

#define PITCH_P_VAL 0.5
#define PITCH_I_VAL 0
#define PITCH_D_VAL 1

#define ROLL_P_VAL 2
#define ROLL_I_VAL 5
#define ROLL_D_VAL 1

#define YAW_P_VAL 2
#define YAW_I_VAL 5
#define YAW_D_VAL 1
/*  GY-85 variables
 *
 */
float ypr[3] = {0.0f, 0.0f, 0.0f};     // yaw pitch roll values
float yprLast[3] = {0.0f, 0.0f, 0.0f};


/* Interrupt lock
 *
 */

boolean interruptLock = false;

/*  Motor controll variables
 *
 */

int velocity;                          // global velocity

float bal_ac, bal_bd;                 // motor balances can vary between -100 & 100
float bal_axes;                       // throttle balance between axes -100:ac , +100:bd

int va, vb, vc, vd;                    //velocities
int v_ac, v_bd;                        // velocity of axes
/*  Filter variables
 *
 */

float ch1Last, ch2Last, ch4Last, velocityLast;
/*  PID variables
 *
 */

PID pitchReg((double *) &ypr[1], (double*)&bal_bd, (double*) &ch2, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, REVERSE);
PID rollReg((double *)&ypr[0], (double *) &bal_ac, (double *) &ch1, ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL, REVERSE);
PID yawReg((double *)&ypr[2], (double *)&bal_axes, (double *) &ch4, YAW_P_VAL, YAW_I_VAL, YAW_D_VAL, DIRECT);



/* Configuracion de Mando PS4
 *
 */

USB Usb;//Declaro la conexion BT con el Mando PS4
BTD Btd(&Usb);
//PS4BT PS4(&Btd, PAIR);
PS4BT PS4(&Btd);

bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value;
bool blinkState = false;


void setup() {

  Serial.begin(115200);//comunicacion serial
  intARM();//Subrutina que carga la velocidad minia a los ESC's
  delay(ESC_ARM_DELAY);
  interrupciones();
  initIMU();
  initBalancing();
  initRegulators();

#if !defined(__MIPSEL__)
  while (!Serial); // Esperando a que el puerto serie se conecte al shield
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\El USB no se inicio"));
    while (1); // FIN
  }

  Serial.print(F("\r\n Inicio Mando PS4"));

}

void loop() {

  //MANDOPS4();//Subrutina Configuracion del mando de PS4
  IMU();  //Subrutina que retorna los valores del sensor
  computePID();
  calculateVelocities();
  updateMotors();

}

void intARM() {
  cuartoESC.attach(ESC_A);// Pin de arduino del ESC 2 es el 7
  tercerESC.attach(ESC_B);// Pin de arduino del ESC 4 es el 5
  primerESC.attach(ESC_C);// Pin de arduino del ESC 3 es el 4
  segundoESC.attach(ESC_D);// Pin de arduino del ESC 1  es el 6
  //ServoCam.attach(SERVOCAM);// Pin de arduino del Servo de la Camara es el 8
  ARM();
}

void ARM() {
  cuartoESC.writeMicroseconds(ESC_MIN);
  tercerESC.writeMicroseconds(ESC_MIN);
  primerESC.writeMicroseconds(ESC_MIN);
  segundoESC.writeMicroseconds(ESC_MIN);
}

void initIMU() {
  Wire.begin();
  accel.initialize();
  mag.initialize();
  gyro.initialize();
}

void IMU() {

  accel.getAcceleration(&ax, &ay, &az);
  mag.getHeading(&mx, &my, &mz);
  gyro.getRotation(&gx, &gy, &gz);
  ypr[0] = ax;
  ypr[1] = ay;
  ypr[2] = az;
  //  Serial.print("Y");
  //  Serial.print(ypr[0]);
  //  Serial.print("\n");
  //  Serial.print("P");
  //  Serial.print(ypr[1]);
  //  Serial.print("\n");
  //  Serial.print("R");
  //  Serial.print(ypr[2]);
  //  Serial.print("\n");
  //      Serial.print("Acelerometro ");
  //      Serial.print(ax); Serial.print(" ");
  //      Serial.print(ay); Serial.print(" ");
  //      Serial.print(az); Serial.print(" ");
}
/*  computePID function
 *
 *  formats data for use with PIDLib
 *  and computes PID output
 */

void computePID() {

  acquireLock();

  ch1 = floor(ch1 / RC_ROUNDING_BASE) * RC_ROUNDING_BASE;
  ch2 = floor(ch2 / RC_ROUNDING_BASE) * RC_ROUNDING_BASE;
  ch4 = floor(ch4 / RC_ROUNDING_BASE) * RC_ROUNDING_BASE;

  ch2 = map(ch2, RC_LOW_CH1, RC_HIGH_CH1, PITCH_MIN, PITCH_MAX);
  ch1 = map(ch1, RC_LOW_CH1, RC_HIGH_CH1, ROLL_MIN, ROLL_MAX);
  ch4 = map(ch4, RC_LOW_CH1, RC_HIGH_CH1, YAW_MIN, YAW_MAX);

  if ((ch2 < PITCH_MIN) || (ch2 > PITCH_MAX)) ch2 = ch2Last;
  if ((ch1 < ROLL_MIN) || (ch1 > ROLL_MAX)) ch1 = ch1Last;
  if ((ch4 < YAW_MIN) || (ch4 > YAW_MAX)) ch4 = ch4Last;

  ch1Last = ch1;
  ch2Last = ch2;
  ch4Last = ch4;

  ypr[0] = ypr[0] * 180 / M_PI;
  ypr[1] = ypr[1] * 180 / M_PI;
  ypr[2] = ypr[2] * 180 / M_PI;

//  if (abs(ypr[0] - yprLast[0]) > 30) ypr[0] = yprLast[0];
//  if (abs(ypr[1] - yprLast[1]) > 30) ypr[1] = yprLast[1];
//  if (abs(ypr[2] - yprLast[2]) > 30) ypr[2] = yprLast[2];
  Serial.print("Y");
  Serial.print(ypr[0]);
  Serial.print("\n");
  Serial.print("P");
  Serial.print(yprLast[0]);
//  Serial.print("\n");
//  Serial.print("R");
//  Serial.print(ypr[2]);
  Serial.print("\n");
  yprLast[0] = ypr[0];
  yprLast[1] = ypr[1];
  yprLast[2] = ypr[2];

  pitchReg.Compute();
  rollReg.Compute();
  yawReg.Compute();

  releaseLock();

}


void calculateVelocities() {

  acquireLock();

  ch3 = floor(ch3 / RC_ROUNDING_BASE) * RC_ROUNDING_BASE;
  velocity = map(ch3, RC_LOW_CH1, RC_HIGH_CH1, ESC_MIN, ESC_MAX);

  releaseLock();

  if ((velocity < ESC_MIN) || (velocity > ESC_MAX)) velocity = velocityLast;

  velocityLast = velocity;

  v_ac = (abs(-100 + bal_axes) / 100) * velocity;
  v_bd = ((100 + bal_axes) / 100) * velocity;

  va = ((100 + bal_ac) / 100) * v_ac;
  vb = ((100 + bal_bd) / 100) * v_bd;

  vc = (abs((-100 + bal_ac) / 100)) * v_ac;
  vd = (abs((-100 + bal_bd) / 100)) * v_bd;
  //  Serial.println("velocity");
  //  Serial.println(velocity);

  if (velocity < ESC_TAKEOFF_OFFSET) {

    va = ESC_MIN;
    vb = ESC_MIN;
    vc = ESC_MIN;
    vd = ESC_MIN;

  }

}

void updateMotors() {

  primerESC.write(va);
  tercerESC.write(vc);
  segundoESC.write(vb);
  cuartoESC.write(vd);
  //    Serial.print(va);
  //    Serial.print("\r\n");
  //    Serial.print(vc);
  //    Serial.print("\r\n");
  //    Serial.print(vb);
  //    Serial.print("\r\n");
  //    Serial.print(vd);
  //    Serial.print("\r\n");

}
void MANDOPS4() {

  Usb.Task();
  int minimo = 850;//valor a proximado para que el cuadricoptero no suba ni baje.
  int value;
  if (PS4.connected()) {
    //Joysticks sin tocar velocidad minima
    if (PS4.getAnalogHat(LeftHatY) >= 110 && PS4.getAnalogHat(LeftHatY) <= 130 && PS4.getAnalogHat(RightHatX) >= 31 && PS4.getAnalogHat(RightHatX) <= 239 && PS4.getAnalogHat(RightHatY) >= 31 && PS4.getAnalogHat(RightHatY) <= 239 && PS4.getAnalogButton(R2) < 240 && PS4.getAnalogButton(L2) < 240) {
      cuartoESC.writeMicroseconds(minimo);
      tercerESC.writeMicroseconds(minimo);
      primerESC.writeMicroseconds(minimo);
      segundoESC.writeMicroseconds(minimo);
      //      Serial.print(F("\t Joysticks sin tocar"));
    }
    //SUBIR Y BAJAR al pulsar el joystick izquierdo mas de valor 137 o igual
    if (PS4.getAnalogHat(LeftHatY) >= 137) {
      value = PS4.getAnalogHat(LeftHatY) * 8, 5;
      cuartoESC.writeMicroseconds(value);
      tercerESC.writeMicroseconds(value);
      primerESC.writeMicroseconds(value);
      segundoESC.writeMicroseconds(value);
      //      Serial.print(F("\tLeftHatY:>=137 "));

    } if (PS4.getAnalogHat(LeftHatY) <= 109) {
      //Para evitar que el rango sea 0 de velocidad al subir y bajar.
      cuartoESC.writeMicroseconds(minimo);
      tercerESC.writeMicroseconds(minimo);
      primerESC.writeMicroseconds(minimo);
      segundoESC.writeMicroseconds(minimo);
      //      Serial.print(F("\tLeftHatY:<=109 "));
    }
    //Roll
    if (PS4.getAnalogHat(RightHatX) <= 30) {
      //        valueR = PS4.getAnalogHat(RightHatX)*8,5;
      cuartoESC.writeMicroseconds(minimo);
      tercerESC.writeMicroseconds(minimo);
      primerESC.writeMicroseconds(minimo);
      segundoESC.writeMicroseconds(1075);
      //      Serial.print(F("\tRightHatX:>=240 "));
    } if (PS4.getAnalogHat(RightHatX) >= 240) {
      //        valueR = PS4.getAnalogHat(RightHatX);
      cuartoESC.writeMicroseconds(1075);
      tercerESC.writeMicroseconds(minimo);
      primerESC.writeMicroseconds(minimo);
      segundoESC.writeMicroseconds(minimo);
      //      Serial.print(F("\tRightHatX:<=30 "));
      //        Serial.println(valueR);

      //Pitch
    } if (PS4.getAnalogHat(RightHatY) <= 30) {
      //        valueP = PS4.getAnalogHat(RightHatY)*8,5;
      cuartoESC.writeMicroseconds(minimo);
      tercerESC.writeMicroseconds(1000);
      primerESC.writeMicroseconds(minimo);
      segundoESC.writeMicroseconds(minimo);
      //      Serial.print(F("\tRightHatY:>=180 "));
    } if (PS4.getAnalogHat(RightHatY) >= 240) {
      //        valueP = PS4.getAnalogHat(RightHatY)*8,5;
      cuartoESC.writeMicroseconds(minimo);
      tercerESC.writeMicroseconds(minimo);
      primerESC.writeMicroseconds(1000);
      segundoESC.writeMicroseconds(minimo);
      //      Serial.print(F("\tRightHatY:<=30 "));

    }
    if (PS4.getAnalogButton(L2) != oldL2Value || PS4.getAnalogButton(R2) != oldR2Value) // Only write value if it's different
      PS4.setRumbleOn(PS4.getAnalogButton(L2), PS4.getAnalogButton(R2));
    oldL2Value = PS4.getAnalogButton(L2);
    oldR2Value = PS4.getAnalogButton(R2);
    if (PS4.getAnalogButton(L2) >= 240) {
      //YAW-->Izquierda
      cuartoESC.writeMicroseconds(minimo);
      tercerESC.writeMicroseconds(minimo);
      primerESC.writeMicroseconds(1000);
      segundoESC.writeMicroseconds(minimo);
      //Serial.print(F("\tYAW Izquierda "));
    }
    if (PS4.getAnalogButton(R2) >= 240) {
      //YAW-->Derecha
      cuartoESC.writeMicroseconds(minimo);
      tercerESC.writeMicroseconds(1175);
      primerESC.writeMicroseconds(minimo);
      segundoESC.writeMicroseconds(minimo);
      //Serial.print(F("\tYAW Derecha "));

    }
    if (PS4.getButtonClick(PS)) {
      //Serial.print(F("\r\nPS"));
      cuartoESC.writeMicroseconds(0);
      tercerESC.writeMicroseconds(0);
      primerESC.writeMicroseconds(0);
      segundoESC.writeMicroseconds(0);
      PS4.disconnect();
      //     }if (PS4.getButtonClick(UP)) {
      //      pos += 1; // goes from 0 degrees to 180 degrees
      //      // in steps of 1 degree
      //      ServoCam.write(pos);              // tell servo to go to position in variable 'pos'
      //      //delay(15);                       // waits 15ms for the servo to reach the position
      //
      //    } if (PS4.getButtonClick(DOWN)) {
      //      pos -= 1; // goes from 180 degrees to 0 degrees
      //      ServoCam.write(pos);              // tell servo to go to position in variable 'pos'
      //      //delay(15);                       // waits 15ms for the servo to reach the position
    }
  }
}

void initBalancing() {

  bal_axes = 0;
  bal_ac = 0;
  bal_bd = 0;

}

void initRegulators() {

  pitchReg.SetMode(AUTOMATIC);
  pitchReg.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);

  rollReg.SetMode(AUTOMATIC);
  rollReg.SetOutputLimits(-PID_ROLL_INFLUENCE, PID_ROLL_INFLUENCE);

  yawReg.SetMode(AUTOMATIC);
  yawReg.SetOutputLimits(-PID_YAW_INFLUENCE, PID_YAW_INFLUENCE);

}

void interrupciones() {

  // Las cuatro interrupciones del mando
  attachInterrupt(PS4.getAnalogHat(LeftHatY), Interrupcion1, CHANGE);
  attachInterrupt(PS4.getAnalogHat(RightHatY), Interrupcion2, CHANGE);
  attachInterrupt(PS4.getAnalogHat(RightHatX), Interrupcion3, CHANGE);
  attachInterrupt(PS4.getAnalogButton(R2), Interrupcion4, CHANGE);
}

void Interrupcion1() {
  if (!interruptLock) ch1 = micros() - UltimoCambio1;
  UltimoCambio1 = micros();
}

void Interrupcion2() {
  if (!interruptLock) ch2 = micros() - UltimoCambio2;
  UltimoCambio2 = micros();
}

void Interrupcion3() {
  if (!interruptLock) ch3 = micros() - UltimoCambio3;
  UltimoCambio3 = micros();
}
void Interrupcion4() {
  if (!interruptLock) ch4 = micros() - UltimoCambio4;
  UltimoCambio4 = micros();
}

void acquireLock() {
  interruptLock = true;
}
void releaseLock() {
  interruptLock = false;
}


