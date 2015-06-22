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

int16_t mx, my, mz;
int16_t gx, gy, gz;

// class default I2C address is 0x53
// specific I2C addresses may be passed as a parameter here
// ALT low = 0x53 (default for SparkFun 6DOF board)
// ALT high = 0x1D
ADXL345 accel;

int16_t ax, ay, az;


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
  initIMU();

  Serial.begin(115200);//comunicacion serial
  Pidroll.setSetPoint(6);
//  Pidpitch.setSetPoint(0);
//  Pidyaw.setSetPoint(0);
  intARM();//Subrutina que carga la velocidad minia a los ESC's

#if !defined(__MIPSEL__)
  while (!Serial); // Esperando a que el puerto serie se conecte al shield
#endif
  if (Usb.Init() == -1) {
    //    Serial.print(F("\r\El USB no se inicio"));
    while (1); // FIN
  }

  //  Serial.print(F("\r\n Inicio Mando PS4"));
}

void loop() {

  MANDOPS4();//Subrutina Configuracion del mando de PS4
  IMU();  //Subrutina que retorna los valores del sensor
//  double roll = ax;
//  double pitch = ay;
//  
  //  double yaw = mx;
  Pidroll.addNewSample(ax);
  //Pidpitch.addNewSample(ay);
  //  Pidyaw.addNewSample(yaw);

  // convierte para el control
//  controlePwm = map(Pidroll.process(), -660, 660, 900, 2000);
//  controlePwm1 = map(Pidpitch.process(), -660, 660, 900, 2000);
  //controlePwm2 = map(Pidyaw.process(), -660, 660, 900, 2000);
//  Updatemotores(controlePwm, controlePwm1);

  //cuartoESC.writeMicroseconds(controlePwm);
  //  Serial.print("ROLL");
//  Serial.print("\r\n ");
  Serial.print(Pidroll.process());
  Serial.print("\r\n ");

  //  //Serial.print(pepe);
  //  Serial.print("\n");
  //  Serial.print("\r\n PITCH");
  //  Serial.print(controlePwm1);
  //  Serial.print("\r\nYAW");
  //  Serial.print(controlePwm2);

  //Updatemotores();
  // SaÃƒÂ­da do controle
  //    Serial.print("\r\n");
  //
  //    Serial.print(" ");
  //    Serial.println(controlePwm);
  delay(100);

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
    if (PS4.getAnalogButton(L2) || PS4.getAnalogButton(R2)) {
      //      Serial.print(F("\r\nL2: "));
      //      Serial.print(PS4.getAnalogButton(L2));
      //      Serial.print(F("\tR2: "));
      //      Serial.print(PS4.getAnalogButton(R2));
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
      //      Serial.print(F("\tYAW Izquierda "));
    }
    if (PS4.getAnalogButton(R2) >= 240) {
      //YAW-->Derecha
      cuartoESC.writeMicroseconds(minimo);
      tercerESC.writeMicroseconds(1175);
      primerESC.writeMicroseconds(minimo);
      segundoESC.writeMicroseconds(minimo);
      //      Serial.print(F("\tYAW Derecha "));

    }
    if (PS4.getButtonClick(PS)) {
      Serial.print(F("\r\nPS"));
      cuartoESC.writeMicroseconds(0);
      tercerESC.writeMicroseconds(0);
      primerESC.writeMicroseconds(0);
      segundoESC.writeMicroseconds(0);
      PS4.disconnect();
    }
  }
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
  //  // verify connection
  //  Serial.println("Testing device connections...");
  //  Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  //  Serial.println("Testing device connections...");
  //  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

  // read raw accel measurements from device


  //    //Serial.print(az);
  //    // display tab-separated accel x/y/z values
//  Serial.print("Acelerometro ");
//  Serial.print(ax); Serial.print(" ");
  //      Serial.print(ay); Serial.print(" ");
  //      Serial.print(az); Serial.print(" ");
  //  //
  //      Serial.print("Magnetometro ");
  //      Serial.print(mx); Serial.print(" ");
  //      Serial.print(my); Serial.print(" ");
  //      Serial.print(mz); Serial.print(" ");
  //
  //      Serial.print("Giroscopio ");
  //      Serial.print(gx); Serial.print(" ");
  //      Serial.print(gy); Serial.print(" ");
  //      Serial.print(az); Serial.print(" ");
  //      Serial.println();
  //    delay(10);

  //    // blink LED to indicate activity
  //    blinkState = !blinkState;
  //    digitalWrite(LED_PIN, blinkState);
  //////////////////////
  ////////////////////
}
void Updatemotores(int roll, int pitch) {


}


