void MANDOPS4() {
  if (PS4.connected()) {
    PS4.setLed(Blue);
    if (PS4.getButtonClick(PS)) {
      primerESC.write(0);
      tercerESC.write(0);
      segundoESC.write(0);
      cuartoESC.write(0);
      PS4.setLed(Red);
      PS4.disconnect();
    }
  }
}
void initPS4(){
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    while (1); // Halt
  }
/*Valores de inicio de los botones
*
*/
PS4.getAnalogHat(LeftHatY)==0;
PS4.getAnalogHat(RightHatY)==0;
PS4.getAnalogHat(RightHatX)==0;
PS4.getAnalogButton(R2)==0;
}

