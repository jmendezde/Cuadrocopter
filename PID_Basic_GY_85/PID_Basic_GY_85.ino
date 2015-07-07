/*Definicion de las librerias para el mando de la PS4
*
*/
#include <PS4BT.h>
#include <usbhub.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

/* Configuracion de Mando PS4
 *
 */
USB Usb;//objeto Usb para utilizar el USB task y encontrar el mando
//USBHub Hub1(&Usb); // algunos doondlges tiene un hub
BTD Btd(&Usb); // La instancia de doodgle

//PS4BT PS4(&Btd, PAIR);//para emparejar por primera vez o cada vez que desemparejemos el mando de la PS4, pulsando Share+PS
PS4BT PS4(&Btd);// Una vez que emparejemos la proximavez que ejecutemos el codigo podremos utilizar este metodo para solo tener que dar al boton PS en vez de  al Share+PS

#include <PID_v1.h>//Libreria PID
#include <Servo.h>//Libreria Servo
#include <Wire.h>//Libreria para conectarse por Wire con los sensores
#include <Kalman.h> //Libreria filtro de Kalman

#define RESTRICT_PITCH

Kalman kalmanX; // Filtro de Kalman X
Kalman kalmanY;// Filtro de Kalman Y
#define gyroAddress 0x68//Direccion de gyroscopio para utilizar por el wire
#define adxlAddress 0x53//Direccion de acelerometro para utilizar por el wire


double zeroValue[5] = { 10, 0, 227, 679, 28}; // valores de los sensores para corregirlos valores obtenidos

/* Valores para el filtro de kalman
*
*/
double gyroXangle = 180;
double gyroYangle = 180;

double compAngleX = 180;
double compAngleY = 180;

double xAngle = 0;
double yAngle = 0;

unsigned long timer;

uint8_t buffer[2]; // I2C buffer
/*Sensor IMU
*
*/
#include "I2Cdev.h"
#include "ADXL345.h"
#include "HMC5883L.h"
#include "ITG3200.h"

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


/*Declaro los sensores del IMU para utilizarlos y sus variables a obtener
*
*/
HMC5883L mag;
ITG3200 gyro;
ADXL345 accel;

int16_t ax, ay, az;
int16_t mx, my, mz;
int16_t gx, gy, gz;


//Se definen las variables a utilizar en le PID
double SetpointRoll, SetpointPitch, SetpointYaw, InputRoll, InputPitch, InputYaw, Output, Output1, Output2;
double ypr[3] = {0, 0, 0};
double yprLast[3] = {0, 0, 0};
double Kp = 2, Ki = 2, Kd = 1;

/*Se declaran la funcion el PID a computar
*
*/

PID rollReg(&ypr[0], &Output, &SetpointRoll, Kp, Ki, Kd, DIRECT);
PID pitchReg(&ypr[1], &Output1, &SetpointPitch, Kp, Ki, Kd, DIRECT);
PID yawReg(&ypr[2], &Output2, &SetpointYaw, Kp, Ki, Kd, DIRECT);

void setup()
{
  //intARM();//Inicializo los valores de los ESC's
  // delay(ESC_ARM_DELAY);------------------------------IMPORTANTE-------------------------------------
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    while (1); // Halt
  }
  //initKalman();//Inicializo los valores de lfiltro de Kalman
  initBalancing();//Inicializo los valores de balanceo
  initRegulators();//Los valores de
  initIMU();//Inicializo el IMU
  initSetpoints();//Inicializo los valores de consigana

}
void loop() {
  Usb.Task();
  MANDOPS4();//Subrutina Configuracion del mando de PS4
  IMU();//Obtencion de los valores del sensor X Y Z
  //KalmanCorrection();//Correccion de los datos del sensor
  computePID();//Se computan las variables del PID
  calculateVelocities();//Se calculan las velocidades
  updateMotors();//Se actualizan la velocidad de los motores
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











