void calculateVelocities(){


  ch3 = floor(150 / RC_ROUNDING_BASE) * RC_ROUNDING_BASE;
  velocity = map(ch3, RC_LOW_CH1, RC_HIGH_CH1, ESC_MAX, ESC_MIN);

  if ((velocity < ESC_MIN) || (velocity > ESC_MAX)) velocity = velocityLast;

  velocityLast = velocity;

  v_ac = (abs(-100 + Output2) / 100) * velocity;
  v_bd = ((100 + Output2) / 100) * velocity;

  va = ((100 + Output) / 100) * v_ac;
  vb = ((100 + Output1) / 100) * v_bd;

  vc = (abs((-100 + Output) / 100)) * v_ac;
  vd = (abs((-100 + Output1) / 100)) * v_bd;

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
