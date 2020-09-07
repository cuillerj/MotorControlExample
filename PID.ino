void TurnPIDOn()
{

  mPID.SetMode(AUTOMATIC);
  mPID.SetOutputLimits(outLimit[mMinOut], outLimit[mMaxOut]);
  SetPIDKx();
  SetPIDSampleTime(delayMiniBetweenHoles / 3);
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
  float speedMotor = Wheels.GetTurnSpeed(mWheelId) * 100;
  mInput = speedMotor;
  mPID.Compute();
  motor.AdjustMotorPWM(int(mOutput));
  timePID = millis();
#if defined(debugPIDOn)
  {
    if (millis() > debugPIDTimer + 50) {
      Serial.print("mIn:");
      Serial.print(mInput);
      Serial.print(" mOU:");
      Serial.print(mOutput);
      Serial.print(" setP:");
      Serial.println(mSetpoint);
      debugPIDTimer = millis();
    }
  }
#endif
#if defined(plotterOn)
  Serial.print(mInput);
  Serial.print("\t");
  Serial.println(mSetpoint);
#endif
}
void SetPIDSampleTime(double sampTime)
{
  mPID.SetSampleTime(sampTime);
}
