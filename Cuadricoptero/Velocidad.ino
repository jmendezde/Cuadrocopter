/*  calculateVelocities function
 *
 *  calculates the velocities of every motor
 *  using the PID output
 */

void calculateVelocities() {

  //acquireLock();
  //ch3 = PS4.getAnalogHat(LeftHatY);
  ch3 = 200;
  // ch3 = floor(190 / RC_ROUNDING_BASE) * RC_ROUNDING_BASE;

  velocity = map(ch3, RC_LOW_CH1, RC_HIGH_CH1, ESC_MIN, ESC_MAX);
  //velocity = map(ch3, RC_LOW_CH1, RC_HIGH_CH1, 22, 80);

  //releaseLock();

  if ((velocity < ESC_MIN) || (velocity > ESC_MAX)) velocity = velocityLast;

  velocityLast = velocity;
  //
  // v_ac = (abs(-100) / 100) * velocity;
  ////      v_bd = ((100) / 100) * velocity;
  ////
  // vb = ((100 + bal_ac) / 100) * v_ac;
  ////      //vb = ((100 + bal_bd) / 100) * v_bd;
  ////
  //vd = (abs((-100 + bal_ac) / 100)) * v_ac;
  ////      //vd = (abs((-100 + bal_bd) / 100)) * v_bd;

  //    v_ac = (abs(-100 + bal_axes) / 100) * velocity;
  //    v_bd = ((100 + bal_axes) / 100) * velocity;
  //
  //      va = ((100 - bal_ac + bal_bd) / 100) * v_ac;
  //      vb = ((100 + bal_ac + bal_bd) / 100) * v_bd;
  //
  //      vc = (abs((-100 + bal_ac- bal_bd) / 100)) * v_ac;
  //      vd = (abs((-100 -bal_ac - bal_bd) / 100)) * v_bd;



  //Xcopter
  //  va = 900 + velocity - (Output) + (Output1);
  //  vb = 900 + velocity + Output + Output1;
  //  vc = 900 + velocity + (Output) - (Output1);
  //  vd = 900 + velocity - (Output) - (Output1);

  //       vb = velocity ;
  //       vd = velocity ;
  //       vc = velocity ;
  //       va = velocity ;

  ////Pitch and Roll
  //      va = velocity - bal_ac + bal_bd;
  //      vb = velocity + bal_ac + bal_bd;
  //      vc = velocity + bal_ac - bal_bd;
  //      vd = velocity - bal_ac - bal_bd;


  //Pitch only
  //va = velocity - bal_bd;
  //  vb = velocity + bal_bd;
  //vc = velocity + bal_bd;
  //  vd = velocity - bal_bd;
  //Roll only
  //vb = velocity + bal_ac;
  ////// vd = velocity + bal_ac;
  ////// vb = velocity + bal_ac;
  //vd = velocity - bal_ac;


  //Xcopter
  va = velocity + bal_ac;
  vd = velocity + bal_ac;
  vb = velocity - bal_ac;
  vc = velocity - bal_ac;
  

  //    if (va < 600) va = 600;
  //    if (vb < 600) vb = 600;
  //    if (vc < 600) vc = 600;
  //    if (vd < 600) vd = 600;
  //    if (va > 2000) va = 2000;
  //    if (vb > 2000) vb = 2000;
  //    if (vc > 2000) vc = 2000;
  //    if (vd > 2000) vd = 2000;

  //
  Serial.print("\t Velocidad ");
  Serial.print(velocity);
  Serial.print("\t bal_ac ");
  Serial.print(bal_ac);

  Serial.print("\t VA");
  Serial.print(va);
  Serial.print("\t VB ");
  Serial.print(vb);
  Serial.print("\t VC ");
  Serial.print(vc);
  Serial.print("\t VD ");
  Serial.println(vd);

  if (velocity < ESC_TAKEOFF_OFFSET) {

    va = ESC_MIN;
    vb = ESC_MIN;
    vc = ESC_MIN;
    vd = ESC_MIN;

  }

}

void updateMotors() {

 // a.writeMicroseconds(va);
//  c.writeMicroseconds(vc);
  b.writeMicroseconds(vb);
  d.writeMicroseconds(vd);

}
void apagadomando() {

  a.writeMicroseconds(600);
  c.writeMicroseconds(600);
  b.writeMicroseconds(600);
  d.writeMicroseconds(600);

}


