#include <PS4BT.h>
#include <usbhub.h>
#include <Servo.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif
//#include <PID_v1.h>
///Sensor IMU
#include "Wire.h"
#include "I2Cdev.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "ITG3200.h"

//Declaro los sensores del IMU
HMC5883L mag;
ITG3200 gyro;
ADXL345 accel;

int16_t ax, ay, az;
int16_t mx, my, mz;
int16_t gx, gy, gz;

// class default I2C address is 0x53
// specific I2C addresses may be passed as a parameter here
// ALT low = 0x53 (default for SparkFun 6DOF board)
// ALT high = 0x1D
double valorPIDroll, valorPIDpitch, valorPIDyaw;
int velocity, velocityLast;                         // global velocity
double ch1, ch2, ch3, ch4;

int va, vb, vc, vd;                    //velocities
int v_ac, v_bd;

/* Configuracion de ESC's
 *
 */
Servo cuartoESC, tercerESC, primerESC, segundoESC; //declaro los servos
#define ESC_A 7
#define ESC_B 6
#define ESC_C 5
#define ESC_D 4

#define ESC_MIN 600
#define ESC_MAX 2000
#define ESC_TAKEOFF_OFFSET 30
#define ESC_ARM_DELAY 5000

/* Configuracion de Mando PS4
 *
 */

USB Usb;//Declaro la conexion BT con el Mando PS4
BTD Btd(&Usb);
//PS4BT PS4(&Btd, PAIR);
PS4BT PS4(&Btd);

bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value;
#define LED_PIN 13 // (Arduino is 13, Teensy is 6)
bool blinkState = false;

/* Configuracion del PID
 *
 */
class PID {

  public:

    double error;
    double sample;
    double lastSample;
    double kP, kI, kD;
    double P, I, D;
    double pid;

    double setPoint;
    double lastProcess;

    PID(double _kP, double _kI, double _kD) {
      kP = _kP;
      kI = _kI;
      kD = _kD;
    }

    void addNewSample(double _sample) {
      sample = _sample;
    }

    void setSetPoint(double _setPoint) {
      setPoint = _setPoint;
    }

    double process() {
      // implementacion del PID
      error = setPoint - sample;
      double deltaTime = (millis() - lastProcess) / 100;
      lastProcess = millis();
      //deltaTime = deltaTime + 0.000001;
      //P
      P = error * kP;

      //I
      I = I + (error * kI) * (deltaTime / 100);

      //D
      D = (lastSample - sample) * kD / deltaTime;
      lastSample = sample;
      // Sumar todo
      pid = P + I + D;

      return pid;
    }
};

PID Pidroll(2.0, 5.0, 1.0);
PID Pidpitch(0.5, 0.0, 1.0);
PID Pidyaw(2.0, 5.00, 1.0);
void setup() {


  Serial.begin(115200);//comunicacion serial

  intARM();//Subrutina que carga la velocidad minia a los ESC's
  initIMU();
  initSetpoint();

}
void loop() {

  //MANDOPS4();//Subrutina Configuracion del mando de PS4
  IMU();  //Subrutina que retorna los valores del sensor
  PIDUpdate();
  Updatemotores();

  // convierte para el control


}

void intARM() {
  cuartoESC.attach(ESC_A);// Pin de arduino del ESC 2 es el 7
  tercerESC.attach(ESC_B);// Pin de arduino del ESC 4 es el 5
  primerESC.attach(ESC_C);// Pin de arduino del ESC 3 es el 4
  segundoESC.attach(ESC_D);// Pin de arduino del ESC 1  es el 6
  ARM();
}

void ARM() {
  cuartoESC.writeMicroseconds(ESC_MIN);
  tercerESC.writeMicroseconds(ESC_MIN);
  primerESC.writeMicroseconds(ESC_MIN);
  segundoESC.writeMicroseconds(ESC_MIN);
  delay(ESC_ARM_DELAY);
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

}
void initSetpoint() {
  Pidroll.setSetPoint(10);
  Pidpitch.setSetPoint(10);
  Pidyaw.setSetPoint(100);
}
void PIDUpdate() {
  Pidroll.addNewSample(ax);
  Pidpitch.addNewSample(ay);
  Pidyaw.addNewSample(my);
  valorPIDroll = map(Pidroll.process(), -255, 255, 900, 2000);
  valorPIDpitch = map(Pidpitch.process(), -255, 255, 900, 2000);
  valorPIDyaw = map(Pidyaw.process(), -660, 660, 900, 2000);
  Serial.print("\tRoll\t");
  Serial.print(Pidroll.process());
  Serial.print("\tPitch\t");
  Serial.print(Pidpitch.process());
  Serial.print("\tYaw\t");
  Serial.print(Pidyaw.process());
  Serial.print("\r\n");

}

void Updatemotores() {
  ch3 = floor(0 / 50) * 50;
  velocity = map(ch3, 0, 255, ESC_MIN, ESC_MAX);

  if ((velocity < ESC_MIN) || (velocity > ESC_MAX)) velocity = velocityLast;

  velocityLast = velocity;

  v_ac = (abs(-100 + valorPIDyaw) / 100) * velocity;
  v_bd = ((100 + valorPIDyaw) / 100) * velocity;

  va = ((100 + valorPIDroll) / 100) * v_ac;
  vb = ((100 + valorPIDpitch) / 100) * v_bd;

  vc = (abs((-100 + valorPIDroll) / 100)) * v_ac;
  vd = (abs((-100 + valorPIDpitch) / 100)) * v_bd;

  if (velocity < 30) {

    va = ESC_MIN;
    vb = ESC_MIN;
    vc = ESC_MIN;
    vd = ESC_MIN;

  }

  primerESC.write(va);
  tercerESC.write(vc);
  segundoESC.write(vb);
  cuartoESC.write(vd);
  //    Serial.print("\tVA\t");
  //  Serial.print(va);
  //  Serial.print("\tVB\t");
  //  Serial.print(vb);
  //  Serial.print("\tVC\t");
  //  Serial.print(vc);
  //  Serial.print("\tVD\t");
  //  Serial.print(vd);
  //  Serial.print("\r\n");
}


