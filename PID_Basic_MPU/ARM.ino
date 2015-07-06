void intARM() {
  cuartoESC.attach(ESC_A);// Pin de arduino del ESC 2 es el 7
  tercerESC.attach(ESC_B);// Pin de arduino del ESC 4 es el 5
  primerESC.attach(ESC_C);// Pin de arduino del ESC 3 es el 4
  segundoESC.attach(ESC_D);// Pin de arduino del ESC 1  es el 6
  //ServoCam.attach(SERVOCAM);// Pin de arduino del Servo de la Camara es el 8
  ARM();
}
void ARM(){ 
  cuartoESC.writeMicroseconds(ESC_MIN);
  tercerESC.writeMicroseconds(ESC_MIN);
  primerESC.writeMicroseconds(ESC_MIN);
  segundoESC.writeMicroseconds(ESC_MIN);
}
