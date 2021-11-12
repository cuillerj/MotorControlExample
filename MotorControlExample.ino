
/*
   This is an example of motor control with a wheel encoder
   turns number and wheel speed can be adjusted with commands (t+,t-) (s+,s-)
   sense of rotation can be selected with command (clkw+,clkw-)
   break or not when turns number is reached can be selected with command (breakOn,breakOff)
      if breakOff holes overflow due to inertia is provided
      if breakOn motor is briefly reversed to reduce inertia effect

   Use serial monitor with or without debugOn and debugPIDOn to get more or less data
   Use serial plotter with plotterOn to get speed graph or plotterOn and debugPIDOn to get both graph speed and PWM

*/

/*
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

  Written by Jean Cuiller
*/


//#define debugOn   // uncomment to get details on the serial link
//#define plotterOn // uncomment to get graph on serial plotter (debugOn must be commented

#define debugPIDOn
//#define SRT
/*
   encoders setup
*/
#include <WheelControl.h>
#define mWheelId 0       // will use the first encoder (wheelControl can support up to 4 encoders)
#define analogEncoderInput A8   // analog input encoder GPIO
#define encoderPower 4   // GPIO used to power on the encoders
//#define maxRPS 4.       // maximum speed rotation rounds per second
//#define minRPS 2.       // minimum speed rotation rounds per second
//#define defaultRPS 3.   // default speed rotation rounds per second
#define maxRPS 2.   // maximum speed rotation rounds per second
#define minRPS 1.     // minimum speed rotation rounds per second
#define defaultRPS 1.5 // default speed rotation rounds per second
#define defaultSetPoint defaultRPS*100  // speed is multiplied by 100 for PID
#define wheelPinInterruptIn 3    // used by sotfware interrupt when rotation reach threshold > wheelPinInterruptIn and wheelPinInterruptOut must be wired together
#define wheelPinInterruptOut 7    // used by sotfware interrupt when rotation reach threshold > wheelPinInterruptIn and wheelPinInterruptOut must be wired together
//#define wheelEncoderHoles 12  // number of holes of the encoder wheel
#define wheelEncoderHoles 30  // number of holes of the encoder wheel
#define delayMiniBetweenHoles  (1000./(maxRPS*wheelEncoderHoles))  //  delay millis second between 2 encoder holes at the maximum speed 
#define delayMaxBetweenHoles  (1000/(minRPS*wheelEncoderHoles))*1.1  //  delay millis second between 2 encoder holes at the minimum s
#define oscilloTrigger 53     // GPIO use to trigger the oscilloscope
boolean breakOn = false;      // true if breaking is needed when threshold is reached
unsigned int breakPulse = 0;  // if breakOn true
#define breakPulseRatio 4 // if breakOn true > need to be fitted with the hardware to adjust breaking
unsigned int incoderHighValue = 500; // (1024 means 5v)  above that signal is high for analogEncoderInput
unsigned int incoderLowValue = 50;  // (1024 means 5v)  below that signal is low for analogEncoderInput
volatile uint8_t wheelIdInterruption = 0xff; // if not 0xff a wheelinterruption has to be analysed
volatile boolean encodersToStop = false;   // flag used to delay stopping encoders after stopping motors
boolean encodersStopped = true;   // encoders status
unsigned long nbHolesRequested;  // nmuber of done revolutions * 100
WheelControl Wheels(wheelEncoderHoles, incoderHighValue, incoderLowValue, analogEncoderInput,   // define encoders: holes number, analog high value, analog low value, analog GPIO for encoder number 0
                    0, 0, 0, 0,
                    0, 0, 0, 0,
                    0, 0, 0, 0,
                    wheelPinInterruptOut, delayMiniBetweenHoles);     // define GPIO interrupt and mimium duration between 2 holes
/*
  motor setup
*/
#include <motorControl.h>
#define motorENA 12 // Arduino pin must be PWM use timer 3
#define motorIN1 26 // arduino pin for rotation control
#define motorIN2 27  // arduino pin for rotation control
#define sizeOfRev 8 // size of the array containing latest revolution wheel speed
long wheeelCumulative = 0;            // cumulative count of the left holes used for dynamic speed adjustment
//unsigned long prevCheckHoles = 0;
unsigned int iMotorMaxrpm = maxRPS * 60;
#define iSlowPWM 20    // under this value the motor will not rotate and the system can enter in a locking state
boolean bClockwise = false; //Need to turn counter-clockwise on left motor to get forward
Motor motor(motorENA, motorIN1, motorIN2, iMotorMaxrpm, iSlowPWM); // define right Motor
/*
   PID setup
*/
#include <PID_v1.h>
#define KpRegister 0   // Kp proportional register position
#define KiRegister 1   // Ki integral register position
#define KdRegister 2   // Kd derivative register position
#define sizeOfKx 3     // register size

float Kx[sizeOfKx] = {1.2, 2., 0.1};  // registers that contain PID Kx
#define mMinOut 0    // 
#define mMaxOut 1
#define mStartOut 2
double mInput, mOutput, mSetpoint;
#define sizeOfOutlim 3
int outLimit[sizeOfOutlim] = {iSlowPWM, 255, 200}; // PWM {minOut,maxOut, startOut}
PID mPID(&mInput, &mOutput, &mSetpoint, Kx[KpRegister], Kx[KiRegister], Kx[KdRegister], DIRECT);
volatile boolean PIDactive = false;
boolean pwmMode = false;
/*
   serial link
*/
uint8_t incomingByte = 0; // for incoming serial data
int incomingByteNomber = 0; // for incoming serial data
uint8_t rawCommand[20];

/*
   timers and counters
*/
unsigned long timePID = 0;
volatile unsigned long timeAfterStopMotors = 0;
unsigned long debugPIDTimer = 0;
unsigned long startMotorTime = 0;
unsigned long stopMotorTime = 0;
unsigned int count = 0;
unsigned long SRTcount = 0;
unsigned long pwmModeTimer = 0;
float lastSpeed = 0;
/*
  serial commands setup
*/
#include <LookForString.h>
#define commandnumber 11
String commandList[commandnumber] = {"start", "stop", "s+", "s-", "brakOn", "brakOff", "t+", "t-", "clkw+", "clkw-", "detGain"};
String *PcommandList[commandnumber];   // pointera array (to each command)
LookForStr SerialInput (PcommandList, commandnumber);   // define the object


void setup() {
  Serial.begin(38400);            // for debugging log
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  delay(200);
  pinMode(encoderPower, OUTPUT);
  digitalWrite(encoderPower, 0);
  pinMode(oscilloTrigger, OUTPUT);
  digitalWrite(oscilloTrigger, LOW);
  delay(100);
  pinMode(wheelPinInterruptIn, INPUT_PULLUP);
  pinMode(wheelPinInterruptOut, OUTPUT);
  nbHolesRequested = 50 * wheelEncoderHoles;
  for (int i = 0; i < commandnumber ; i++)
  {
    PcommandList[i] = &commandList[i];
  }
  SerialInput.InitCommandsList(PcommandList, commandnumber);
  mSetpoint = defaultSetPoint;
#if defined(plotterOn)
#if defined(debugPIDOn)
  Serial.print("mOut");
  Serial.print("\t");
  Serial.print("speed");
  Serial.print("\t");
  Serial.println("mSetpoint");
#endif
#endif
#if defined(plotterOn)
#if !defined(debugPIDOn)
  Serial.print("Speed");
  Serial.print("\t");
  Serial.println("setpoint");
#endif
#endif
#ifndef plotterOn
  Serial.print("commands:");
  for (int i = 0; i < commandnumber; i++) {
    Serial.print(" ");
    Serial.print(commandList[i]);
  }
  Serial.println();
#endif
}

void loop() {
  delay(1);
  count++;
  if (wheelIdInterruption != 0xff)                   // we got wheel encoder interruption
  {
    WheelThresholdReached(wheelIdInterruption);                      // call the threshold analyse
#if defined(debugOn)
    Serial.print("wheels interrupt: ");
    Serial.println(wheelIdInterruption);
#endif
    wheelIdInterruption = 0xff;                      // clear the flag
  }
  if (!encodersStopped && count > 1000) {
    count = 0;
#if defined(debugOn)
    Serial.print(Wheels.GetWheelThreshold(mWheelId));
    Serial.print(" ");
    Serial.print(Wheels.GetCurrentHolesCount(mWheelId));
    Serial.print(" min level: ");
    Serial.print(Wheels.GetMinLevel(mWheelId));
    Serial.print(" maxlevel: ");
    Serial.print(Wheels.GetMaxLevel(mWheelId));
    Serial.print(" ");
    Serial.print(Wheels.GetWheeLowValue(mWheelId));
    Serial.print(" ");
    Serial.println(Wheels.GetWheeHighValue(mWheelId));
#endif

  }
  if (!encodersStopped && encodersToStop && millis() > timeAfterStopMotors + 1000) {
    StopEncoders();
  }
  if (PIDactive)
  {
    ComputePID();
  }
  if (Serial.available() > 0) {
    ReadCommand();
  }
#if defined SRT
  unsigned int h = 0;
  unsigned int m = 0;
  unsigned int s = 0;
  unsigned long result = 0;
  if (millis() - startMotorTime >  SRTcount * 1000 && !encodersStopped) {
    h = SRTcount / 3600;
    result = SRTcount - h * 3600;
    m = result / 60;
    s = SRTcount - m * 60;
    Serial.println(SRTcount);
    Serial.print(h);
    Serial.print(":");
    Serial.print(m);
    Serial.print(":");
    Serial.print(s);
    SRTcount++;
    h = SRTcount / 3600;
    result = SRTcount - h * 3600;
    m = result / 60;
    s = SRTcount - m * 60;
    Serial.print(",000 --> ");
    Serial.print(h);
    Serial.print(":");
    Serial.print(m);
    Serial.print(":");
    Serial.println(s);
    Serial.print(SRTcount);
    Serial.println(" sec");
    Serial.println();

  }
#endif
  if (pwmMode && millis() > pwmModeTimer + 100) {
    Serial.println(Wheels.GetTurnSpeed(mWheelId) * 100);
    pwmModeTimer = millis();
    if (millis() > startMotorTime + 10000) {
      pwmMode = false;
      StopMotors();
    }
  }
}
