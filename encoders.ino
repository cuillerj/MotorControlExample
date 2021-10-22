void WheelInterrupt()   // wheel controler set a software interruption due to threshold reaching
{
  // detachInterrupt(digitalPinToInterrupt(wheelPinInterruptIn));
  wheelIdInterruption = Wheels.GetLastWheelInterruptId();         // get which wheel Id reached the threshold to be analysed in the next loop
  #if defined(debugOn)
  Serial.println(wheelIdInterruption);
  #endif
  Wheels.ClearThreshold(wheelIdInterruption);                      // clear the threshold flag to avoid any more interruption
  PIDactive = false;
  //  attachInterrupt(digitalPinToInterrupt(wheelPinInterruptIn), WheelInterrupt, FALLING);
}
void WheelThresholdReached( uint8_t wheelId)
{
#if defined(debugOn)
  Serial.print("reach:");
  Serial.println(wheelId);
#endif
  if (wheelId != 5)
  {
    StopMotors();
    digitalWrite(oscilloTrigger, LOW);
    timeAfterStopMotors = millis();
    encodersToStop = true;                               // to keep wheel ecoders running a little bit
    if (breakOn) {
    //  ReverseMotors();
            BreakMotors();
    }
    else {
      StopMotors();
    }
#if defined(debugOn)
    Serial.println("encoders to be stopped");
#endif
  }
  else                      // wheel mode pulse
  {
    StopMotors();
  }

}
void StopEncoders()
{
#if defined(debugOn)
  Serial.println("stopping encoders");
#endif

  encodersToStop = false;                             // set flag encoder to be stopped a little bit later to take into account inertia
  Wheels.StopWheelControl(true, true, false, false);  // stop wheel control
  detachInterrupt(digitalPinToInterrupt(wheelPinInterruptIn));
  digitalWrite(encoderPower, LOW);
#ifndef plotterOn
  Serial.print("total holes:");
  Serial.print(Wheels.GetCurrentHolesCount(mWheelId));
  if (!breakOn) {
    Serial.print(" overflow holes:");
    Serial.println( Wheels.GetCurrentHolesCount(mWheelId) - nbHolesRequested);
  }
#endif
  wheeelCumulative = wheeelCumulative + Wheels.GetCurrentHolesCount(mWheelId);
  unsigned int holes = Wheels.GetCurrentHolesCount(mWheelId);
  encodersStopped = true;
  digitalWrite(encoderPower, 0);
  float avgRPS = nbHolesRequested / wheelEncoderHoles; // turns number
  float duration = (float(stopMotorTime) - startMotorTime) / 1000;
  avgRPS = (avgRPS / duration);
#ifndef plotterOn
  Serial.print(" duration:");
  Serial.print(duration);
  Serial.print("sec >> average RPS:");
  Serial.println(avgRPS);
#endif

}
void StartEncoders() {
#if defined(debugOn)
  Serial.println("starts encoders");
#endif
  digitalWrite(encoderPower, 1);
  attachInterrupt(digitalPinToInterrupt(wheelPinInterruptIn), WheelInterrupt, FALLING);
  digitalWrite(oscilloTrigger, HIGH);
  Wheels.StartWheelControl(true, true, nbHolesRequested , false, false, 0 , false, false, 0, false, false, 0);
  delay(10);
  encodersStopped = false;
}
