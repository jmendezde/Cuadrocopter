void initRC() {
  pinMode(RC_PWR, OUTPUT);
  digitalWrite(RC_PWR, HIGH);

  // FIVE FUCKING INTERRUPTS !!!
  attachInterrupt(RC_1, rcInterrupt1, CHANGE);
  attachInterrupt(RC_2, rcInterrupt2, CHANGE);
  attachInterrupt(RC_3, rcInterrupt3, CHANGE);
  attachInterrupt(RC_4, rcInterrupt4, CHANGE);
  attachInterrupt(RC_5, rcInterrupt5, CHANGE);

}

void rcInterrupt1() {
  if (!interruptLock) ch1 = micros() - rcLastChange1;
  rcLastChange1 = micros();
}

void rcInterrupt2() {
  if (!interruptLock) ch2 = micros() - rcLastChange2;
  rcLastChange2 = micros();
}

void rcInterrupt3() {
  if (!interruptLock) ch3 = micros() - rcLastChange3;
  rcLastChange3 = micros();
}

void rcInterrupt4() {
  if (!interruptLock) ch4 = micros() - rcLastChange4;
  rcLastChange4 = micros();
}

void rcInterrupt5() {
  if (!interruptLock) ch5 = micros() - rcLastChange5;
  rcLastChange5 = micros();
}

void acquireLock() {
  interruptLock = true;
}

void releaseLock() {
  interruptLock = false;
}
