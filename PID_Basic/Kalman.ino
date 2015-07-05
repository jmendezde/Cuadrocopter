

void KalmanCorrection() {

  double gyroXrate = -(((double)readGyroX() - zeroValue[3]) / 14.375);
  
  gyroXangle += gyroXrate * ((double)(micros() - timer) / 1000000); // Without any filter

  double gyroYrate = (((double)readGyroY() - zeroValue[4]) / 14.375);
  gyroYangle += gyroYrate * ((double)(micros() - timer) / 1000000); // Without any filter

  double accXangle = getXangle();
  double accYangle = getYangle();

  compAngleX = (0.93 * (compAngleX + (gyroXrate * (double)(micros() - timer) / 1000000))) + (0.07 * accXangle);
  compAngleY = (0.93 * (compAngleY + (gyroYrate * (double)(micros() - timer) / 1000000))) + (0.07 * accYangle);

  xAngle = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer)); // calculate the angle using a Kalman filter
  yAngle = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros() - timer)); // calculate the angle using a Kalman filter

  timer = micros();

  /* print data to processing */
//  Serial.print(gyroXangle); Serial.print("\t");
//  Serial.print(gyroYangle); Serial.print("\t");
//
//  Serial.print(accXangle); Serial.print("\t");
//  Serial.print(accYangle); Serial.print("\t");
//
//  Serial.print(compAngleX); Serial.print("\t");
//  Serial.print(compAngleY); Serial.print("\t");

  Serial.print(xAngle); Serial.print("\t");
  Serial.print(yAngle); Serial.print("\t");
  ypr[0] = xAngle;
  ypr[1] = yAngle;
  ypr[2] = my;
//  Serial.print("\n");

  delay(10);
}

int readGyroX() { // This really measures the y-axis of the gyro
  return (gx);
}
int readGyroY() { // This really measures the x-axis of the gyro
   return (gy);
}
double getXangle() {
  double accXval = (double)readAccX() - zeroValue[0];
  double accZval = (double)readAccZ() - zeroValue[2];

  double angle = (atan2(accXval, accZval) + PI) * RAD_TO_DEG;
  return angle;
}
double getYangle() {
  double accYval = (double)readAccY() - zeroValue[1];
  double accZval = (double)readAccZ() - zeroValue[2];
 
  double angle = (atan2(accYval, accZval) + PI) * RAD_TO_DEG;
  return angle;
}
int readAccX() {

  return (ax);
}
int readAccY() {
  return (ay);
}
int readAccZ() {
  return (az);
}

void initKalman() {
 
  kalmanX.setAngle(90); // Set starting angle
  kalmanY.setAngle(90);
  timer = micros();
}

