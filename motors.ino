void StartMotors()
{
  TurnPIDOn();   // turn PID on
  //prevCheckHoles = 0;
  if (nbHolesRequested > 0)
  {
#ifndef plotterOn
    Serial.print(" start motor > turns number:");
    Serial.print(nbHolesRequested / wheelEncoderHoles);
    Serial.print(" holes number:");
    Serial.print(nbHolesRequested);
    Serial.print(" RPS:");
    float speedReq = mSetpoint / 100;
    Serial.println(speedReq);
#endif
    //  mSaveWheelInterrupt = 0;
    double startPWM = outLimit[mStartOut];
    SRTcount=1;
    startMotorTime = millis();
    motor.RunMotor(bClockwise,  startPWM);
  }
}
void StopMotors()
{
  breakPulse = mSetpoint / breakPulseRatio;
  motor.StopMotor();
#if defined(debugOn)
  Serial.println("motor stopped");
#endif
  stopMotorTime = millis();
  PIDactive = false;
  timePID = millis();
}
void ReverseMotors()
{
  Wheels.StartWheelPulse(breakPulse);
  motor.RunMotor(!bClockwise,  255);
}
