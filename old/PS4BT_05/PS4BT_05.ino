#include <PS4BT.h>
#include <usbhub.h>
#include <Servo.h>
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

Servo cuartoESC,tercerESC,primerESC,segundoESC; //declaro los servos
// ESC configuration
#define ESC_MIN 600
#define ESC_MAX 2000
#define ESC_TAKEOFF_OFFSET 30
#define ESC_ARM_DELAY 5000

//  Filter variables
float ch1Last, ch2Last, ch4Last, velocityLast;
//RC configuration
#define RC_HIGH_CH1 1000
#define RC_LOW_CH1 2000
#define RC_HIGH_CH2 1000
#define RC_LOW_CH2 2000
#define RC_HIGH_CH3 1000
#define RC_LOW_CH3 2000
#define RC_HIGH_CH4 1000
#define RC_LOW_CH4 2000
#define RC_HIGH_CH5 1000
#define RC_LOW_CH5 2000
#define RC_ROUNDING_BASE 50
/* Interrupt lock
 *
 */
boolean interruptLock = false;
// RC channel inputs
float ch1, ch2, ch3, ch4, ch5;
unsigned long rcLastChange1 = micros();
unsigned long rcLastChange2 = micros();
unsigned long rcLastChange3 = micros();
unsigned long rcLastChange4 = micros();
unsigned long rcLastChange5 = micros();

//  PID configuration
#define PITCH_P_VAL 0.5
#define PITCH_I_VAL 0
#define PITCH_D_VAL 1
#define ROLL_P_VAL 2
#define ROLL_I_VAL 5
#define ROLL_D_VAL 1
#define YAW_P_VAL 2
#define YAW_I_VAL 5
#define YAW_D_VAL 1
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
/*  PID variables
 *
 */
int velocity;                          // global velocity
float bal_ac, bal_bd;                 // motor balances can vary between -100 & 100
float bal_axes;                       // throttle balance between axes -100:ac , +100:bd
int va, vb, vc, vd;                    //velocities
int v_ac, v_bd;                        // velocity of axes
float ypr[3] = {0.0f,0.0f,0.0f};       // yaw pitch roll values
float yprLast[3] = {0.0f, 0.0f, 0.0f};
PID pitchReg(&ypr[1], &bal_bd, &ch2, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, REVERSE);
PID rollReg(&ypr[2], &bal_ac, &ch1, ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL, REVERSE);
PID yawReg(&ypr[0], &bal_axes, &ch4, YAW_P_VAL, YAW_I_VAL, YAW_D_VAL, DIRECT);
//Declaro la conexion BT con el Mando PS4
USB Usb;
BTD Btd(&Usb); 
//PS4BT PS4(&Btd, PAIR);
PS4BT PS4(&Btd);
bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value;
#define LED_PIN 13 // (Arduino is 13, Teensy is 6)
bool blinkState = false;
void setup() {
  Serial.begin(115200);//comunicacion serial
  cuartoESC.attach(7);// Pin de arduino del ESC 2 es el 7
  tercerESC.attach(5);// Pin de arduino del ESC 4 es el 5
  primerESC.attach(4);// Pin de arduino del ESC 3 es el 4
  segundoESC.attach(6);// Pin de arduino del ESC 1  es el 6
  arm();
  initRC();
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
  MANDOPS4();//Funcion Configuracion del mando de PS4
  IMU();  //Funcion que retorna los valores del sensor
  CPUPID();
  calculateVelocities();
  updateMotors();
}  


void MANDOPS4(){
Usb.Task();
int value = 600; 
  if (PS4.connected()) {
  //Joysticks sin tocar velocidad minima
  if (PS4.getAnalogHat(LeftHatY) >= 110 && PS4.getAnalogHat(LeftHatY) <= 130 && PS4.getAnalogHat(RightHatX) >= 31 && PS4.getAnalogHat(RightHatX) <= 239 && PS4.getAnalogHat(RightHatY)>=31 && PS4.getAnalogHat(RightHatY)<=239&& PS4.getAnalogButton(R2)<240 && PS4.getAnalogButton(L2)<240){
        cuartoESC.writeMicroseconds(850);
        tercerESC.writeMicroseconds(850);
        primerESC.writeMicroseconds(850);
        segundoESC.writeMicroseconds(850);
        Serial.print(F("\t Joysticks sin tocar"));
 }   
      //SUBIR Y BAJAR al pulsar el joystick izquierdo mas de valor 137 o igual
     if (PS4.getAnalogHat(LeftHatY)>=137){
        value = PS4.getAnalogHat(LeftHatY)*8,5;
        cuartoESC.writeMicroseconds(value);
        tercerESC.writeMicroseconds(value);
        primerESC.writeMicroseconds(value);
        segundoESC.writeMicroseconds(value);
        Serial.print(F("\tLeftHatY:>=137 "));

   }if (PS4.getAnalogHat(LeftHatY)<=109){
     //Para evitar que el rango sea 0 de velocidad al subir y bajar.
        cuartoESC.writeMicroseconds(800);
        tercerESC.writeMicroseconds(800);
        primerESC.writeMicroseconds(800);
        segundoESC.writeMicroseconds(800);
        Serial.print(F("\tLeftHatY:<=109 "));
   }
     //Roll
    if(PS4.getAnalogHat(RightHatX)<=30){
//        valueR = PS4.getAnalogHat(RightHatX)*8,5;
        cuartoESC.writeMicroseconds(850);
        tercerESC.writeMicroseconds(850);
        primerESC.writeMicroseconds(850);
        segundoESC.writeMicroseconds(1075);
        Serial.print(F("\tRightHatX:>=240 ")); 
     }if(PS4.getAnalogHat(RightHatX)>=240){
//        valueR = PS4.getAnalogHat(RightHatX);
        cuartoESC.writeMicroseconds(1075);
        tercerESC.writeMicroseconds(850);
        primerESC.writeMicroseconds(850);
        segundoESC.writeMicroseconds(850);
        Serial.print(F("\tRightHatX:<=30 ")); 
//        Serial.println(valueR);  
     
   //Pitch
     }if(PS4.getAnalogHat(RightHatY)<=30){
//        valueP = PS4.getAnalogHat(RightHatY)*8,5;
        cuartoESC.writeMicroseconds(850);
        tercerESC.writeMicroseconds(1000);
        primerESC.writeMicroseconds(850);
        segundoESC.writeMicroseconds(850);
               Serial.print(F("\tRightHatY:>=180 ")); 
     }if(PS4.getAnalogHat(RightHatY)>=240){
//        valueP = PS4.getAnalogHat(RightHatY)*8,5;
        cuartoESC.writeMicroseconds(850);
        tercerESC.writeMicroseconds(850);
        primerESC.writeMicroseconds(1000);
        segundoESC.writeMicroseconds(850);
        Serial.print(F("\tRightHatY:<=30 ")); 
       
     }
     if (PS4.getAnalogButton(L2) || PS4.getAnalogButton(R2)) { 
      Serial.print(F("\r\nL2: "));
      Serial.print(PS4.getAnalogButton(L2));
      Serial.print(F("\tR2: "));
      Serial.print(PS4.getAnalogButton(R2));
    }
    if (PS4.getAnalogButton(L2) != oldL2Value || PS4.getAnalogButton(R2) != oldR2Value) // Only write value if it's different
      PS4.setRumbleOn(PS4.getAnalogButton(L2), PS4.getAnalogButton(R2));
    oldL2Value = PS4.getAnalogButton(L2);
    oldR2Value = PS4.getAnalogButton(R2);
if (PS4.getAnalogButton(L2)>=240){
 //YAW-->Izquierda
        cuartoESC.writeMicroseconds(850);
        tercerESC.writeMicroseconds(850);
        primerESC.writeMicroseconds(1000);
        segundoESC.writeMicroseconds(850);
        Serial.print(F("\tYAW Izquierda ")); 
}
if(PS4.getAnalogButton(R2)>=240){
      //YAW-->Derecha
        cuartoESC.writeMicroseconds(850);
        tercerESC.writeMicroseconds(1175);
        primerESC.writeMicroseconds(850);
        segundoESC.writeMicroseconds(850); 
        Serial.print(F("\tYAW Derecha ")); 
}
    if (PS4.getButtonClick(PS)) {
      Serial.print(F("\r\nPS"));
      PS4.disconnect();
    }
//      if (PS4.getButtonClick(L1)){  
//       Serial.print(F("\r\nL1"));
//     } if (PS4.getButtonClick(L3))
//        Serial.print(F("\r\nL3"));
//      if (PS4.getButtonClick(R1)){
//       Serial.print(F("\r\nR1"));
//     } if (PS4.getButtonClick(R3))
//        Serial.print(F("\r\nR3"));
//
//       if (PS4.getButtonClick(SHARE))
//        Serial.print(F("\r\nShare"));
//      if (PS4.getButtonClick(OPTIONS)) {
//        Serial.print(F("\r\nOptions"));
//        printAngle = !printAngle;
//      }
//      if (PS4.getButtonClick(TOUCHPAD)) {
//        Serial.print(F("\r\nTouchpad"));
//        printTouch = !printTouch;
//      }
//
//      if (printAngle) { // Imprime el angulo cuando tenemos activado el acelerometro
//        Serial.print(F("\r\nPitch: "));
//        Serial.print(PS4.getAngle(Pitch));
//        Serial.print(F("\tRoll: "));
//        Serial.print(PS4.getAngle(Roll));
//      }
//      if (printTouch) { // Imprime la x e y del touchpad
//        if (PS4.isTouching(0) || PS4.isTouching(1)) // Imprimir línea nueva y retorno de carro si alguno de los dedos estén tocando la pantalla táctil
//          Serial.print(F("\r\n"));
//        for (uint8_t i = 0; i < 2; i++) { // trackeo de dos dedos
//          if (PS4.isTouching(i)) { // imprime la posicion de dedo al tocar el touchpad
//            Serial.print(F("X")); Serial.print(i + 1); Serial.print(F(": "));
//            Serial.print(PS4.getX(i));
//            Serial.print(F("\tY")); Serial.print(i + 1); Serial.print(F(": "));
//            Serial.print(PS4.getY(i));
//            Serial.print(F("\t"));
//          }
        }
      }
 

void IMU(){
    Wire.begin();// join I2C bus (I2Cdev library doesn't do this automatically)  
//    Serial.println("Initializing I2C devices...");// initialize device
    accel.initialize();
//    Serial.println("Testing device connections...");
//    Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");
    pinMode(LED_PIN, OUTPUT);// configure LED for output
    mag.initialize();
    gyro.initialize();
//    Serial.println("Testing device connections...");
//    Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
    accel.getAcceleration(&ax, &ay, &az);// read raw accel measurements from device
    mag.getHeading(&mx, &my, &mz);
    gyro.getRotation(&gx, &gy, &gz);   
//    Serial.print("Acelerometro ");//Serial.print(az);
//    Serial.print(ax); Serial.print(" "); // display tab-separated accel x/y/z values
//    Serial.print(ay); Serial.print(" ");
//    Serial.print(az); Serial.print(" ");
//    Serial.print("Magnetometro ");
//    Serial.print(mx); Serial.print(" ");
//    Serial.print(my); Serial.print(" ");
//    Serial.print(mz); Serial.print(" ");
//    Serial.print("Giroscopio ");
//    Serial.print(gx); Serial.print(" ");
//    Serial.print(gy); Serial.print(" ");
//    Serial.print(az); Serial.print(" ");
//    Serial.println();
} 
void CPUPID(){
  acquireLock();
  ch1 = floor(ch1/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  ch2 = floor(ch2/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  ch4 = floor(ch4/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  ch2 = map(ch2, RC_LOW_CH2, RC_HIGH_CH2, PITCH_MIN, PITCH_MAX);
  ch1 = map(ch1, RC_LOW_CH1, RC_HIGH_CH1, ROLL_MIN, ROLL_MAX);
  ch4 = map(ch4, RC_LOW_CH4, RC_HIGH_CH4, YAW_MIN, YAW_MAX);
  if((ch2 < PITCH_MIN) || (ch2 > PITCH_MAX)) ch2 = ch2Last;
  if((ch1 < ROLL_MIN) || (ch1 > ROLL_MAX)) ch1 = ch1Last;
  if((ch4 < YAW_MIN) || (ch4 > YAW_MAX)) ch4 = ch4Last;
  ch1Last = ch1;
  ch2Last = ch2;
  ch4Last = ch4;
  ypr[0] = ypr[0] * 180/M_PI;
  ypr[1] = ypr[1] * 180/M_PI;
  ypr[2] = ypr[2] * 180/M_PI;
  if(abs(ypr[0]-yprLast[0])>30) ypr[0] = yprLast[0];
  if(abs(ypr[1]-yprLast[1])>30) ypr[1] = yprLast[1];
  if(abs(ypr[2]-yprLast[2])>30) ypr[2] = yprLast[2];
  yprLast[0] = ypr[0];
  yprLast[1] = ypr[1];
  yprLast[2] = ypr[2];
  pitchReg.Compute();
  rollReg.Compute();
  yawReg.Compute();
  releaseLock();
}
void calculateVelocities(){//  calculateVelocities function,calculates the velocities of every motor,using the PID output
  acquireLock();
  ch3 = floor(ch3/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  velocity = map(ch3, RC_LOW_CH3, RC_HIGH_CH3, ESC_MIN, ESC_MAX);
  releaseLock();
  if((velocity < ESC_MIN) || (velocity > ESC_MAX)) velocity = velocityLast;
  velocityLast = velocity;
  v_ac = (abs(-100+bal_axes)/100)*velocity;
  v_bd = ((100+bal_axes)/100)*velocity;
  va = ((100+bal_ac)/100)*v_ac;
  vb = ((100+bal_bd)/100)*v_bd;
  vc = (abs((-100+bal_ac)/100))*v_ac;
  vd = (abs((-100+bal_bd)/100))*v_bd;
  Serial.println(bal_bd);
  if(velocity < ESC_TAKEOFF_OFFSET){
    va = ESC_MIN;
    vb = ESC_MIN;
    vc = ESC_MIN;
    vd = ESC_MIN;
  }
}
void initRC(){
//  pinMode(RC_PWR, OUTPUT);
//  digitalWrite(RC_PWR, HIGH);
  
  // FIVE FUCKING INTERRUPTS !!!
  PCintPort::attachInterrupt(PS4.getAnalogHat(LeftHatY), rcInterrupt1, CHANGE);
  PCintPort::attachInterrupt(PS4.getAnalogHat(RightHatY), rcInterrupt2, CHANGE);
  PCintPort::attachInterrupt(PS4.getAnalogHat(RightHatX), rcInterrupt3, CHANGE);
  PCintPort::attachInterrupt(PS4.getAnalogButton(L2), rcInterrupt4, CHANGE);
  PCintPort::attachInterrupt(PS4.getAnalogButton(R2), rcInterrupt5, CHANGE);
  
}
void initBalancing(){

  bal_axes = 0;
  bal_ac = 0;
  bal_bd = 0;

}
void arm(){
  primerESC.write(ESC_MIN),
  segundoESC.write(ESC_MIN),
  tercerESC.write(ESC_MIN),
  cuartoESC.write(ESC_MIN);
  delay(ESC_ARM_DELAY);
}
void updateMotors(){
  segundoESC.write(va),
  cuartoESC.write(vc),
  primerESC.write(vb),
  tercerESC.write(vd);
}
void acquireLock(){
  interruptLock = true; 
}
void releaseLock(){
  interruptLock = false;
}

void rcInterrupt1(){
   if(!interruptLock) ch1 = micros() - rcLastChange1;
   rcLastChange1 = micros(); 
}

void rcInterrupt2(){
  if(!interruptLock) ch2 = micros() - rcLastChange2;
  rcLastChange2 = micros();
}

void rcInterrupt3(){
  if(!interruptLock) ch3 = micros() - rcLastChange3;
  rcLastChange3 = micros();
}

void rcInterrupt4(){
  if(!interruptLock) ch4 = micros() - rcLastChange4;
  rcLastChange4 = micros();
}

void rcInterrupt5(){
  if(!interruptLock) ch5 = micros() - rcLastChange5;
  rcLastChange5 = micros();
}
