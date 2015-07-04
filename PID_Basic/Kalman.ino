

void KalmanCorrection() {

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(ay, az) * RAD_TO_DEG;
  double pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;

#else // Eq. 28 and 29
  double roll  = atan(ay / sqrt(ax * ax + az * az)) * RAD_TO_DEG;
  double pitch = atan2(-ax, az) * RAD_TO_DEG;

#endif

  double gxrate = gx / 131.0; // Convert to deg/s
  double gyrate = gy / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gxangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gxrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyrate = -gyrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyrate, dt);

#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyangle = pitch;

  } else
    kalAngleY = kalmanY.getAngle(pitch, gyrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gxrate = -gxrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gxrate, dt); // Calculate the angle using a Kalman filter
#endif

  gxangle += gxrate * dt; // Calculate gyro angle without any filter
  gyangle += gyrate * dt;
  //gxangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gxrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gxangle < -180 || gxangle > 180)
    gxangle = kalAngleX;
  if (gyangle < -180 || gyangle > 180)
    gyangle = kalAngleY;

  /* Print Data */
  //#if 0 // Set to 1 to activate
  //  Serial.print(ax); Serial.print("\t");
  //  Serial.print(ay); Serial.print("\t");
  //  Serial.print(az); Serial.print("\t");
  //
  //  Serial.print(gx); Serial.print("\t");
  //  Serial.print(gy); Serial.print("\t");
  //  Serial.print(gz); Serial.print("\t");
  //
  //  Serial.print("\t");
  //#endif

//  Serial.print("Roll  "); Serial.print(roll); Serial.print("\t");
//  Serial.print("GXANGLE  "); Serial.print(gxangle); Serial.print("\t");
//  Serial.print("CompAngelX  "); Serial.print(compAngleX); Serial.print("\t");
//  Serial.print("KalAngelX"); Serial.print(kalAngleX); Serial.print("\t");
  ypr[1] = kalAngleX;
//  Serial.print("<-->  ");
//
//  Serial.print("Pitch"); Serial.print(pitch); Serial.print("\t");
//  Serial.print("Gyangel"); Serial.print(gyangle); Serial.print("\t");
//  Serial.print("compAngelY"); Serial.print(compAngleY); Serial.print("\t");
//  Serial.print("KalAngleY"); Serial.print(kalAngleY); Serial.print("\t");

  //#if 0 // Set to 1 to print the temperature
  //  Serial.print("\t");
  //
  //  double temperature = (double)tempRaw / 340.0 + 36.53;
  //  Serial.print(temperature); Serial.print("\t");
  //#endif

  Serial.print("\r\n");
  delay(2);
}
void initKalman(){
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(ay, az) * RAD_TO_DEG;
  double pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(ay / sqrt(ax * ax + az * az)) * RAD_TO_DEG;
  double pitch = atan2(-ax, az) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gxangle = roll;
  gyangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

