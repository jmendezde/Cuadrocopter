void MANDOPS4() {
  if (PS4.connected()) {
    if (PS4.getButtonClick(PS)) {
      primerESC.write(0);
      tercerESC.write(0);
      segundoESC.write(0);
      cuartoESC.write(0);
      PS4.disconnect();
    }
  }
}

