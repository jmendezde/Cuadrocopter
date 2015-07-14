/////*Definicion de las librerias para el mando de la PS4
////*
////*/
//#include <PS4BT.h>
//#include <usbhub.h>
//#ifdef dobogusinclude
//#include <spi4teensy3.h>
//#include <SPI.h>
//#endif
//
///* Configuracion de Mando PS4
// *
// */
//USB Usb;//objeto Usb para utilizar el USB task y encontrar el mando
////USBHub Hub1(&Usb); // algunos doondlges tiene un hub
//BTD Btd(&Usb); // La instancia de doodgle
//
////PS4BT PS4(&Btd, PAIR);//para emparejar por primera vez o cada vez que desemparejemos el mando de la PS4, pulsando Share+PS
//PS4BT PS4(&Btd);// Una vez que emparejemos la proximavez que ejecutemos el codigo podremos utilizar este metodo para solo tener que dar al boton PS en vez de  al Share+PS

#include <PID_v1.h>//Libreria PID
#include <Servo.h>//Libreria Servo
#include <Wire.h>
#include <helper_3dmath.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

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

volatile bool mpuInterrupt = false;    //interrupt flag

/*valores del pitch roll y yaw maximo y minimo
*
*/
#define PITCH_MIN -150
#define PITCH_MAX 150
#define ROLL_MIN -150
#define ROLL_MAX 150
#define YAW_MIN -150
#define YAW_MAX 150
#define PID_PITCH_INFLUENCE 100
#define PID_ROLL_INFLUENCE 100
#define PID_YAW_INFLUENCE 100

/*valores de los ESC maximos y tiempo de delay para iniciar los ESC's
*
*/
#define ESC_MIN 600
#define ESC_MAX 1900
#define ESC_TAKEOFF_OFFSET 30
#define ESC_ARM_DELAY 10

/* Valores de entrada del mando que se filtrna para copararlos con los anteriores
 *
 */
double ch1, ch2, ch3, ch4;
double ch1Last, ch2Last, ch4Last, velocityLast;
int velocity;                          // global velocity

int va, vb, vc, vd;                    //velocities para los motores
int v_cd, v_ab, v_cb, v_ad;                      // velocity of axes
Servo cuartoESC, tercerESC, primerESC, segundoESC; //declaro los servos
/*Se defino os pines de salidas de los ESC
*
*/
#define ESC_A 7
#define ESC_B 6
#define ESC_C 5
#define ESC_D 4
/* PS4 valores de configuración
 *
 */

#define RC_HIGH_CH1 255
#define RC_LOW_CH1 0
#define RC_ROUNDING_BASE 50

/* Interrupt lock
 *
 */
boolean interruptLock = false;
unsigned long rcLastChange3 = micros();

//Se definen las variables a utilizar en le PID
float SetpointRoll, SetpointPitch, SetpointYaw, InputRoll, InputPitch, InputYaw, Output, Output1, Output2;
double Kp = 2, Ki = 2, Kd = 1;

/*Se declaran la funcion el PID a computar
*
*/

PID rollReg(&ypr[0], &Output, &SetpointRoll, Kp, Ki, Kd, DIRECT);
PID pitchReg(&ypr[1], &Output1, &SetpointPitch, Kp, Ki, Kd, DIRECT);
PID yawReg(&ypr[2], &Output2, &SetpointYaw, Kp, Ki, Kd, DIRECT);

void setup()
{
  intARM();//Inicializo los valores de los ESC's
  // delay(ESC_ARM_DELAY);------------------------------IMPORTANTE-------------------------------------
  initMPU();//Inicializo los valores de lfiltro de Kalman
  //initPS4();
  initBalancing();//Inicializo los valores de balanceo
  initRegulators();//Los valores de

  initSetpoints();//Inicializo los valores de consigana

}
void loop() {
  //Usb.Task();
  Serial.begin(115200);
  getYPR();//Correccion de los datos del sensor
  //MANDOPS4();//Subrutina Configuracion del mando de PS4
  computePID();//Se computan las variables del PID
  calculateVelocities();//Se calculan las velocidades
  updateMotors();//Se actualizan la velocidad de los motores
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

void rcInterrupt3(){
   if(!interruptLock) ch3 = micros() - rcLastChange3;
   rcLastChange3 = micros(); 
}


void acquireLock(){
  interruptLock = true; 
}

void releaseLock(){
  interruptLock = false;
}










