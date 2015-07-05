void computePID() {

  rollReg.Compute();
  pitchReg.Compute();
  yawReg.Compute();

  //  ch1 = floor(Output / RC_ROUNDING_BASE) * RC_ROUNDING_BASE;
  //  ch2 = floor(Output1 / RC_ROUNDING_BASE) * RC_ROUNDING_BASE;
  //  ch4 = floor(Output2 / RC_ROUNDING_BASE) * RC_ROUNDING_BASE;
  ch1 = Output;
  ch2 = Output1;
  ch4 = Output2;
  Serial.print("\tCH1\t");
  Serial.print(ch1);
  Serial.print("\tCH2\t");
  Serial.print(ch2);
  Serial.print("\tCH4\t");
  Serial.print(ch4);
  Serial.print("\t");
  ch2 = map(ch2, 0, PID_PITCH_INFLUENCE, -PITCH_MIN, PITCH_MAX);
  ch1 = map(ch1, 0, PID_ROLL_INFLUENCE, -ROLL_MIN, ROLL_MAX);
  ch4 = map(ch4, 0, PID_YAW_INFLUENCE, -YAW_MIN, YAW_MAX);

  if ((ch2 < PITCH_MIN) || (ch2 > PITCH_MAX)) ch2 = ch2Last;
  if ((ch1 < ROLL_MIN) || (ch1 > ROLL_MAX)) ch1 = ch1Last;
  if ((ch4 < YAW_MIN) || (ch4 > YAW_MAX)) ch4 = ch4Last;

  ch1Last = ch1;
  ch2Last = ch2;
  ch4Last = ch4;

//     ch1 = ch1 * 180 / M_PI;
//     ch2 = ch2 * 180 / M_PI;
//     ch4 = ch4 * 180 / M_PI;

  //    ypr[0] = ypr[0] * 180 / M_PI;
  //    ypr[1] = ypr[1] * 180 / M_PI;
  //    ypr[2] = ypr[2] * 180 / M_PI;

  //  if (abs(ypr[0] - yprLast[0]) > 30) ypr[0] = yprLast[0];
  //  if (abs(ypr[1] - yprLast[1]) > 30) ypr[1] = yprLast[1];
  //  if (abs(ypr[2] - yprLast[2]) > 30) ypr[2] = yprLast[2];

  yprLast[0] = ypr[0];
  yprLast[1] = ypr[1];
  yprLast[2] = ypr[2];

}

void initRegulators() {

  pitchReg.SetMode(AUTOMATIC);
  pitchReg.SetOutputLimits(0, PID_PITCH_INFLUENCE);
  //pitchReg.SetSampleTime(19);

  rollReg.SetMode(AUTOMATIC);
  rollReg.SetOutputLimits(0, PID_ROLL_INFLUENCE);
  //rollReg.SetSampleTime(19);

  yawReg.SetMode(AUTOMATIC);
  yawReg.SetOutputLimits(0, PID_YAW_INFLUENCE);
  //yawReg.SetSampleTime(19);
}

void initBalancing() {

  Output2 = 0;
  Output = 0;
  Output1 = 0;

}
void initSetpoints() {
  SetpointRoll = 10;
  SetpointPitch = 0;
  SetpointYaw = -155;
}

