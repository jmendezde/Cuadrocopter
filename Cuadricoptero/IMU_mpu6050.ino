///*  getYPR function
// *
// *  gets data from MPU and
// *  computes pitch, roll, yaw on the MPU's DMP
// */
//
//void getYPR() {
//
//  mpuInterrupt = false;
//  mpuIntStatus = mpu.getIntStatus();
//  fifoCount = mpu.getFIFOCount();
//
//  if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {
//
//    mpu.resetFIFO();
//
//  } else if (mpuIntStatus & 0x02) {
//
//    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
//
//    mpu.getFIFOBytes(fifoBuffer, packetSize);
//
//    fifoCount -= packetSize;
//
//    mpu.dmpGetQuaternion(&q, fifoBuffer);
//    mpu.dmpGetGravity(&gravity, &q);
//    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//
//  }
//
//}
//
//void initMPU() {
//
//  Wire.begin();
//  mpu.initialize();
//  devStatus = mpu.dmpInitialize();
//  if (devStatus == 0) {
//
//    mpu.setDMPEnabled(true);
//    attachInterrupt(0, dmpDataReady, RISING);
//    mpuIntStatus = mpu.getIntStatus();
//    packetSize = mpu.dmpGetFIFOPacketSize();
//
//  }
//}
//
//void dmpDataReady() {
//  mpuInterrupt = true;
//}


