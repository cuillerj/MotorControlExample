void TurnPIDOn()
{
  pwmMode = false;
  mPID.SetMode(AUTOMATIC);
  mPID.SetOutputLimits(outLimit[mMinOut], outLimit[mMaxOut]);
  SetPIDKx();
  SetPIDSampleTime(3 * delayMiniBetweenHoles);
  PIDactive = true;
#if defined(debugOn)
  {
    Serial.print("minOut;");
    Serial.print(outLimit[mMinOut]);
    Serial.print(" maxOut;");
    Serial.println(outLimit[mMaxOut]);
  }
#endif
}
void SetPIDKx()
{
  mPID.SetTunings(Kx[KpRegister], Kx[KiRegister], Kx[KdRegister]);
#if defined(debugOn)
  {
    Serial.print(" Kp:");
    Serial.print(mPID.GetKp());
    Serial.print(" Ki:");
    Serial.print(mPID.GetKi());
    Serial.print(" Kd:");
    Serial.println(mPID.GetKd());
    Serial.print("minOut;");
    Serial.print(outLimit[mMinOut]);
  }
#endif
}

void ComputePID()
{
  if (millis() >= timePID +  25000. / (mSetpoint))  // update speed every 1/4 turn
  {
    mInput = Wheels.GetTurnSpeed(mWheelId) * 100;
  }
  mPID.Compute();
  motor.AdjustMotorPWM(round(mOutput));

#if defined(debugPIDOn)
#ifndef plotterOn
  if (millis() >= timePID +  25000. / (mSetpoint))
  {
    //   if (millis() > debugPIDTimer + 50) {
    Serial.print("mIn:");
    Serial.print(mInput);
    Serial.print(" mOU:");
    Serial.print(mOutput);
    Serial.print(" setP:");
    Serial.println(mSetpoint);
    debugPIDTimer = millis();
    //
  }
#endif
#endif
#if defined(plotterOn)
  if (millis() >= timePID +  25000. / (mSetpoint))
  {
#if defined(debugPIDOn)
    Serial.print(mOutput);
    Serial.print("\t");
    Serial.print(mInput);
    Serial.print("\t");
    Serial.println(mSetpoint);
#else
    Serial.print(mInput);
    Serial.print("\t");
    Serial.println(mSetpoint);
#endif;
    timePID = millis();
  }
#endif
  }
  void SetPIDSampleTime(double sampTime)
  {
    mPID.SetSampleTime(sampTime);
  }
