/*  computePID function
 *
 *  formats data for use with PIDLib
 *  and computes PID output
 */

void computePID() {

  //acquireLock();
  ch1 = 0;
  ch2 = 0;
  ch4 = 0;
  ch1 = floor(ch1 / RC_ROUNDING_BASE) * RC_ROUNDING_BASE;
  ch2 = floor(ch2 / RC_ROUNDING_BASE) * RC_ROUNDING_BASE;
  ch4 = floor(ch4 / RC_ROUNDING_BASE) * RC_ROUNDING_BASE;

  ch2 = map(ch2, RC_LOW_CH2, RC_HIGH_CH2, PITCH_MIN, PITCH_MAX);
  ch1 = map(ch1, RC_LOW_CH1, RC_HIGH_CH1, ROLL_MIN, ROLL_MAX);
  ch4 = map(ch4, RC_LOW_CH4, RC_HIGH_CH4, YAW_MIN, YAW_MAX);

  if ((ch2 < PITCH_MIN) || (ch2 > PITCH_MAX)) ch2 = ch2Last;
  if ((ch1 < ROLL_MIN) || (ch1 > ROLL_MAX)) ch1 = ch1Last;
  if ((ch4 < YAW_MIN) || (ch4 > YAW_MAX)) ch4 = ch4Last;

  ch1Last = ch1;
  ch2Last = ch2;
  ch4Last = ch4;

  ypr[0] = ypr[0] * 180 / M_PI;
  ypr[1] = ypr[1] * 180 / M_PI;
  ypr[2] = ypr[2] * 180 / M_PI;

  if (abs(ypr[0] - yprLast[0]) > 30) ypr[0] = yprLast[0];
  if (abs(ypr[1] - yprLast[1]) > 30) ypr[1] = yprLast[1];
  if (abs(ypr[2] - yprLast[2]) > 30) ypr[2] = yprLast[2];

  yprLast[0] = ypr[0];
  yprLast[1] = ypr[1];
  yprLast[2] = ypr[2];
  Serial.print("Y 0  ");
  Serial.print(ypr[0]);
  Serial.print("  R 1  ");
  Serial.print(ypr[1]);
  Serial.print("  P 2  ");
  Serial.print(ypr[2]);
  pitchReg.Compute();
  rollReg.Compute();
  yawReg.Compute();
  Serial.print("   BD  ");
  Serial.print(bal_bd);
  Serial.print("  AC  ");
  Serial.print(bal_ac);
  Serial.print("  AXES  ");
  Serial.print(bal_axes);
  //releaseLock();

}

void initRegulators() {

  pitchReg.SetMode(AUTOMATIC);
  pitchReg.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);
  
  rollReg.SetMode(AUTOMATIC);
  rollReg.SetOutputLimits(-PID_ROLL_INFLUENCE, PID_ROLL_INFLUENCE);

  yawReg.SetMode(AUTOMATIC);
  yawReg.SetOutputLimits(-PID_YAW_INFLUENCE, PID_YAW_INFLUENCE);

}


void initBalancing() {

  bal_axes = 0;
  bal_ac = 0;
  bal_bd = 0;

}


