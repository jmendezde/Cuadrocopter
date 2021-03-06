void computePID() {

  ch1 = floor(SetpointRoll / RC_ROUNDING_BASE) * RC_ROUNDING_BASE;
  ch2 = floor(SetpointPitch / RC_ROUNDING_BASE) * RC_ROUNDING_BASE;
  ch4 = floor(SetpointYaw / RC_ROUNDING_BASE) * RC_ROUNDING_BASE;

  ch2 = map(ch2, RC_LOW_CH1, RC_HIGH_CH1, PITCH_MIN, PITCH_MAX);
  ch1 = map(ch1, RC_LOW_CH1, RC_HIGH_CH1, ROLL_MIN, ROLL_MAX);
  ch4 = map(ch4, RC_LOW_CH1, RC_HIGH_CH1, YAW_MIN, YAW_MAX);

  if ((ch2 < PITCH_MIN) || (ch2 > PITCH_MAX)) ch2 = ch2Last;
  if ((ch1 < ROLL_MIN) || (ch1 > ROLL_MAX)) ch1 = ch1Last;
  if ((ch4 < YAW_MIN) || (ch4 > YAW_MAX)) ch4 = ch4Last;

  ch1Last = ch1;
  ch2Last = ch2;
  ch4Last = ch4;

//  ypr[0] = InputRoll * 180 / M_PI;
//  ypr[1] = InputPitch * 180 / M_PI;
//  ypr[2] = InputYaw * 180 / M_PI;

  if (abs(ypr[0] - yprLast[0]) > 30) ypr[0] = yprLast[0];
  if (abs(ypr[1] - yprLast[1]) > 30) ypr[1] = yprLast[1];
  if (abs(ypr[2] - yprLast[2]) > 30) ypr[2] = yprLast[2];

  yprLast[0] = ypr[0];
  yprLast[1] = ypr[1];
  yprLast[2] = ypr[2];

  rollReg.Compute();
  pitchReg.Compute();
  yawReg.Compute();
}

void initRegulators() {

  pitchReg.SetMode(AUTOMATIC);
  pitchReg.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);
  //pitchReg.SetSampleTime(19);

  rollReg.SetMode(AUTOMATIC);
  rollReg.SetOutputLimits(-PID_ROLL_INFLUENCE, PID_ROLL_INFLUENCE);
  //rollReg.SetSampleTime(19);

  yawReg.SetMode(AUTOMATIC);
  yawReg.SetOutputLimits(-PID_YAW_INFLUENCE, PID_YAW_INFLUENCE);
  //yawReg.SetSampleTime(19);
}

void initBalancing() {

  Output2 = 0;
  Output = 0;
  Output1 = 0;

}
void initSetpoints(){
  SetpointRoll = 30;
  SetpointPitch = 0;
  SetpointYaw = 107;
}

