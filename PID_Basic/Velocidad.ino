void calculateVelocities() {


  ch3 = floor(0 / RC_ROUNDING_BASE) * RC_ROUNDING_BASE;
  velocity = map(ch3, RC_LOW_CH1, RC_HIGH_CH1, 850, 2000);
  //velocity = map(ch3, RC_LOW_CH1, RC_HIGH_CH1, ESC_MIN, ESC_MAX);
  //  if ((velocity < ESC_MIN) || (velocity > ESC_MAX)) velocity = velocityLast;
  //
  //  velocityLast = velocity;
  Serial.print("velocidad\t");
  Serial.print(velocity);

  v_cd = (abs(-100 + ch1) / 100) * velocity;
  v_ab = ((100 + ch1) / 100) * velocity;

  v_ad = (abs(-100 + ch1) / 100);
  v_cb = ((100 + ch1) / 100);

  vc = ((100 + ch2) / 100) * v_ab;
  vd = ((100 + ch1) / 100) * v_ab;

  //  va = (((100 + ch2) / 100)) * v_cd;
  //vb = (((100 + ch2) / 100)) * v_cd;
  va = (abs((-100 + ch2) / 100)) * v_cd;
  vb = (abs((-100 + ch1) / 100)) * v_cd;
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
