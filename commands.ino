void ReadCommand() {
  // read the incoming byte:
  //  incomingByte = Serial.read();
  rawCommand[incomingByteNomber] = incomingByte;
  incomingByteNomber++;
  String Srequest = (Serial.readString());
  commandReturn rc = SerialInput.GetCommand(Srequest); // look for commant inside the input
  int cmdIdx = rc.idxCommand;
  int cmdPos = rc.idxPos;
  incomingByteNomber = 0;
  switch (cmdIdx) {
    case -1:
      Serial.println("invalid command");
      break;
    case 0:
      TurnPIDOn();   // turn PID on
      StartEncoders();
      StartMotors();
      break;
    case 1:
      StopEncoders();
      motor.StopMotor();
      break;
    case 2:
      if (mSetpoint < 100 * maxRPS) {
        mSetpoint = mSetpoint + 10;
      }
      Serial.print("speed+:");
      Serial.println(mSetpoint);
      break;
    case 3:
      if (mSetpoint > 100 * minRPS) {
        mSetpoint = mSetpoint - 10;
      }
      Serial.print("speed-:");
      Serial.println(mSetpoint);
      break;
    case 4:
      breakOn = true;
      Serial.println("set break on");
      break;
    case 5:
      breakOn = false;
      Serial.println("set break off");
      break;
    case 6:
      nbHolesRequested = nbHolesRequested + 10 * wheelEncoderHoles;
      Serial.print(" turn number+:");
      Serial.println(nbHolesRequested / wheelEncoderHoles);
      break;
    case 7:
      if (nbHolesRequested > 2 * 10 * wheelEncoderHoles) {
        nbHolesRequested = nbHolesRequested - 10 * wheelEncoderHoles;
      }
      Serial.print(" turn number-:");
      Serial.println(nbHolesRequested / wheelEncoderHoles);
      break;
    case 8:
      bClockwise = true;
      Serial.print(" clockwise:");
      Serial.println(bClockwise);
      break;
    case 9:
      bClockwise = false;
      Serial.print(" clockwise:");
      Serial.println(bClockwise);
      break;
    case 10:
      Serial.println(" detGain");
      StartPWM();
      break;
  }
  if (incomingByteNomber >= sizeof(rawCommand)) {
    Serial.println("too long input");
    incomingByteNomber = 0;
    return;
  }
}
