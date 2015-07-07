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
  while (!Serial); // Wait for serial port to connect - used on UNO R3, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    while (1); // Halt
  }
}

