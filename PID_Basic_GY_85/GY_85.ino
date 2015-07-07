void initIMU() {
  Wire.begin();
  accel.initialize();
  mag.initialize();
  gyro.initialize();
  ypr[0] = 0;
  ypr[1] = 0;
  ypr[2] = 0;
}

void IMU() {

  accel.getAcceleration(&ax, &ay, &az);
  mag.getHeading(&mx, &my, &mz);
  gyro.getRotation(&gx, &gy, &gz);
  ypr[0] = ax;
  ypr[1] = ay;
  ypr[2] = my;
//  Serial.print("\tax-");
//  Serial.print(ax);
//  Serial.print("\tay-");
//  Serial.print(ay);
//  Serial.print("\tmy-");
//  Serial.print(my);
//  Serial.print("  ");

}
