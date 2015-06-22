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




/* ESC configuration
 *
 */

#define ESC_MIN 600
#define ESC_MAX 2000
#define ESC_TAKEOFF_OFFSET 30
#define ESC_ARM_DELAY 5000

/* Interrupt lock
 *
 */
 
boolean interruptLock = false;



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
   
  static int value = 600;
  static int valueP = 600;
  static int valueR = 600;
  static int valueY = 600;

  cuartoESC.writeMicroseconds(600);
  tercerESC.writeMicroseconds(600);
  primerESC.writeMicroseconds(600);
  segundoESC.writeMicroseconds(600);
  
  MANDOPS4();//Funcion Configuracion del mando de PS4
  IMU();  //Funcion que retorna los valores del sensor

  
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
    else {
      if (PS4.getButtonClick(TRIANGLE)) {
        Serial.print(F("\r\nTraingle"));
        PS4.setRumbleOn(RumbleLow);
      }
      if (PS4.getButtonClick(CIRCLE)) {
        Serial.print(F("\r\nCircle"));
        PS4.setRumbleOn(RumbleHigh);
      }
      if (PS4.getButtonClick(CROSS)) {
        Serial.print(F("\r\nCross"));
        PS4.setLedFlash(10, 10); //Parpadeo rapido
      }
      if (PS4.getButtonClick(SQUARE)) {
        Serial.print(F("\r\nSquare"));
        PS4.setLedFlash(0, 0); // Apagar el parpadeo
      }

      if (PS4.getButtonClick(UP)) {
        Serial.print(F("\r\nUp"));
        PS4.setLed(Red);
      } if (PS4.getButtonClick(RIGHT)) {
        Serial.print(F("\r\nRight"));
        PS4.setLed(Blue);
      } if (PS4.getButtonClick(DOWN)) {
        Serial.print(F("\r\nDown"));
        PS4.setLed(Yellow);
      } if (PS4.getButtonClick(LEFT)) {
        Serial.print(F("\r\nLeft"));
        PS4.setLed(Green);
      }

      if (PS4.getButtonClick(L1)){  
       Serial.print(F("\r\nL1"));
     } if (PS4.getButtonClick(L3))
        Serial.print(F("\r\nL3"));
      if (PS4.getButtonClick(R1)){
       Serial.print(F("\r\nR1"));
     } if (PS4.getButtonClick(R3))
        Serial.print(F("\r\nR3"));

       if (PS4.getButtonClick(SHARE))
        Serial.print(F("\r\nShare"));
      if (PS4.getButtonClick(OPTIONS)) {
        Serial.print(F("\r\nOptions"));
        printAngle = !printAngle;
      }
      if (PS4.getButtonClick(TOUCHPAD)) {
        Serial.print(F("\r\nTouchpad"));
        printTouch = !printTouch;
      }

      if (printAngle) { // Imprime el angulo cuando tenemos activado el acelerometro
        Serial.print(F("\r\nPitch: "));
        Serial.print(PS4.getAngle(Pitch));
        Serial.print(F("\tRoll: "));
        Serial.print(PS4.getAngle(Roll));
      }

      if (printTouch) { // Imprime la x e y del touchpad
        if (PS4.isTouching(0) || PS4.isTouching(1)) // Imprimir línea nueva y retorno de carro si alguno de los dedos estén tocando la pantalla táctil
          Serial.print(F("\r\n"));
        for (uint8_t i = 0; i < 2; i++) { // trackeo de dos dedos
          if (PS4.isTouching(i)) { // imprime la posicion de dedo al tocar el touchpad
            Serial.print(F("X")); Serial.print(i + 1); Serial.print(F(": "));
            Serial.print(PS4.getX(i));
            Serial.print(F("\tY")); Serial.print(i + 1); Serial.print(F(": "));
            Serial.print(PS4.getY(i));
            Serial.print(F("\t"));
          }
        }
      }
  }
}
}
void IMU(){
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize device
    Serial.println("Initializing I2C devices...");
    accel.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    mag.initialize();
    gyro.initialize();

    Serial.println("Testing device connections...");
    Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
 
// read raw accel measurements from device
accel.getAcceleration(&ax, &ay, &az);
mag.getHeading(&mx, &my, &mz);
gyro.getRotation(&gx, &gy, &gz);

    //Serial.print(az);
    // display tab-separated accel x/y/z values
    Serial.print("Acelerometro ");
    Serial.print(ax); Serial.print(" ");
    Serial.print(ay); Serial.print(" ");
    Serial.print(az); Serial.print(" ");
    
    Serial.print("Magnetometro ");
    Serial.print(mx); Serial.print(" ");
    Serial.print(my); Serial.print(" ");
    Serial.print(mz); Serial.print(" ");
    
    Serial.print("Giroscopio ");
    Serial.print(gx); Serial.print(" ");
    Serial.print(gy); Serial.print(" ");
    Serial.print(az); Serial.print(" ");
    Serial.println();
////    delay(10);

//    // blink LED to indicate activity
//    blinkState = !blinkState;
//    digitalWrite(LED_PIN, blinkState);
//////////////////////
////////////////////  
} 



