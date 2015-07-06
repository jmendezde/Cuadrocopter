void calculateVelocities() {


  ch3 = floor(0 / RC_ROUNDING_BASE) * RC_ROUNDING_BASE;
  velocity = map(ch3, RC_LOW_CH1, RC_HIGH_CH1, 850, 1500);
  //velocity = map(ch3, RC_LOW_CH1, RC_HIGH_CH1, ESC_MIN, ESC_MAX);
  if (velocity < 850) velocity = 850;
  if (velocity > 2000) va = 2000;

  if ((velocity < ESC_MIN) || (velocity > ESC_MAX)) velocity = velocityLast;

  velocityLast = velocity;
  Serial.print("velocidad\t");
  Serial.print(velocity);
  //Xcopter
  //  vb = velocity + Output + Output1 - Output2;
  //  vc = velocity + Output - Output1 + Output2 ;
  //  vd = velocity - Output - Output1 + Output2 ;
  //  va = velocity - Output + Output1 + Output2 ;
  //Quadcopter
  va = velocity - Output1 - Output2 ;
  vb = velocity + Output1 - Output2;
  vc = velocity - Output  + Output2 ;
  vd = velocity + Output  + Output2 ;

  //  va = velocity + Output1 - Output2;
  //  vc = velocity - Output1 - Output2;
  //  vd = velocity + Output + Output2;
  //  vb = velocity - Output + Output2;

  //  v_cd = (abs(-100 + ch4) / 100) * velocity;
  //  v_ab = ((100 + ch4) / 100) * velocity;
  //
  //  v_ad = (abs(-100 + ch1) / 100);
  //  v_cb = ((100 + ch1) / 100);

  //  vc = ((100 + ch2) / 100) * v_cd;
  //  vd = ((100 + ch1) / 100) * v_ab;

  //  va = (((100 + ch2) / 100)) * v_cd;
  //vb = (((100 + ch2) / 100)) * v_cd;
  //
  //  if (ch1 > 0)va = (ch1 + velocity); vb = (ch1 + velocity);
  //  vc = (velocity - ch1); vd = (velocity - ch1);
  //
  //  if (ch1 < 0)va = (ch1 - velocity); vb = (ch1 - velocity);
  //  vc = (velocity + ch1); vd = (velocity + ch1);

  //  v_ab = (abs(-100 + Output2) / 100) * velocity;
  //  v_cd = ((100 + Output2) / 100) * velocity;
  //
  //  va = ((100 + Output) / 100) * v_ab;
  //  vb = ((100 + Output1) / 100) * v_cd;
  //
  //  vc = (abs((-100 + Output) / 100)) * v_cd;
  //  vd = (abs((-100 + Output1) / 100)) * v_cd;

  if (va < 850) va = 850;
  if (vb < 850) vb = 850;
  if (vc < 850) vc = 850;
  if (vd < 850) vd = 850;
  if (va > 2000) va = 2000;
  if (vb > 2000) vb = 2000;
  if (vc > 2000) vc = 2000;
  if (vd > 2000) vd = 2000;

  //  Serial.print("\tv_ac\t");
  //  Serial.print(v_ac);
  //  Serial.print("v_bd\t");
  //  Serial.print(v_bd);
  //
  //  Serial.print("\tOutpu2\t");
  //  Serial.print(Output2);
  //  Serial.print("Outpu\t");
  //  Serial.print(Output);
  //  Serial.print("Outpu1\t");
  //  Serial.print(Output1);

  if (velocity < ESC_TAKEOFF_OFFSET) {

    va = ESC_MIN;
    vb = ESC_MIN;
    vc = ESC_MIN;
    vd = ESC_MIN;

  }

}
void updateMotors() {
  primerESC.write(va);
  tercerESC.write(vc);
  segundoESC.write(vb);
  cuartoESC.write(vd);
}
