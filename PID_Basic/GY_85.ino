void initIMU(){ 
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

//  InputRoll = ax;
//  InputPitch = ay;
//  InputYaw = my;
  
//  ypr[0] = gx;
//  ypr[1] = gy;
//  ypr[2] = gz;
//
//  InputRoll = ypr[0];
//  InputPitch = ypr[1];
//  InputYaw = ypr[2];

//  Serial.print("AX:\t");
//  Serial.print(ypr[0]);
//  Serial.print("  AY:\t");
//  Serial.print(ypr[1]);
//  Serial.print("  AZ:\t");
//  Serial.print(ypr[2]);
}
