#include <PS4BT.h>
#include <usbhub.h>
#include <Servo.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

Servo cuartoESC,tercerESC,primerESC,segundoESC; //declaro los servos

USB Usb;
BTD Btd(&Usb); 
//PS4BT PS4(&Btd, PAIR);
PS4BT PS4(&Btd);
int pos=0;
bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value;
Servo ServoCam;
void setup() {
  cuartoESC.attach(7);// Pin de arduino del ESC 2 es el 8
  tercerESC.attach(5);// Pin de arduino del ESC 4 es el 5
  primerESC.attach(4);// Pin de arduino del ESC 3 es el 4
  segundoESC.attach(6);// Pin de arduino del ESC 1  es el 6
  Serial.begin(115200);
   ServoCam.attach(8);
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
  Usb.Task();
  static int value = 600;
  static int valueP = 600;
  static int valueR = 600;
  static int valueY = 600;
//  PS4.getAnalogHat(RightHatX)==0;
//  PS4.getAnalogHat(LeftHatY)==0;
//  PS4.getAnalogHat(RightHatY)==0;
//  PS4.getAnalogHat(LeftHatX)==0;
  cuartoESC.writeMicroseconds(600);
  tercerESC.writeMicroseconds(600);
  primerESC.writeMicroseconds(600);
  segundoESC.writeMicroseconds(600);
  if (PS4.connected()) {
////      Serial.print(F("\r\nLeftHatX: "));
////      Serial.print(PS4.getAnalogHat(LeftHatX));
////      Serial.print(F("\tLeftHatY: "));
////      Serial.print(PS4.getAnalogHat(LeftHatY));
////      Serial.print(F("\tRightHatX: "));
////      Serial.print(PS4.getAnalogHat(RightHatX));
////      Serial.print(F("\tRightHatY: "));
////      Serial.print(PS4.getAnalogHat(RightHatY));

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
        Serial.print(value);

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

      }if (PS4.getButtonClick(UP)) {
      pos += 1; // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      ServoCam.write(pos);              // tell servo to go to position in variable 'pos'
      //delay(15);                       // waits 15ms for the servo to reach the position

    } if (PS4.getButtonClick(DOWN)) {
      pos -= 1; // goes from 180 degrees to 0 degrees
      ServoCam.write(pos);              // tell servo to go to position in variable 'pos'
      //delay(15);                       // waits 15ms for the servo to reach the position
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

