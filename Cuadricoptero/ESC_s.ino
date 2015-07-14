void initESCs() {

  a.attach(ESC_A);
  b.attach(ESC_B);
  c.attach(ESC_C);
  d.attach(ESC_D);

  delay(100);

  arm();

}
void arm() {

  a.write(ESC_MIN);
  b.write(ESC_MIN);
  c.write(ESC_MIN);
  d.write(ESC_MIN);

  delay(ESC_ARM_DELAY);

}
