# Arduino PID library - with non-uniform time sampling support

This PID library is based on Brett Beauregard's
[Arduino PID library](https://github.com/br3ttb/Arduino-PID-Library)
adapted to work when time between samples is not constant.

## Example
```
/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_Timed.h>

#define PIN_INPUT 0
#define PIN_OUTPUT 3

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, 0.03, DIRECT); // 30 msec target average sampling rate

void setup()
{
  //initialize the variables we're linked to
  Input = analogRead(PIN_INPUT);
  Setpoint = 100;

  // Set PID output limits if necessary
  // myPID.SetOutputLimits(-1, 1);
  
  myPID.enable(true);
}

void loop()
{
  delay(10);
  Input = analogRead(PIN_INPUT);
  myPID.Compute(0.01);  // 10 msec elapsed since last sample
  analogWrite(PIN_OUTPUT, Output);

  delay(50);
  Input = analogRead(PIN_INPUT);
  myPID.Compute(0.05);  // 50 msec elapsed since last sample
  analogWrite(PIN_OUTPUT, Output);

  delay(35);
  Input = analogRead(PIN_INPUT);
  myPID.Compute(0.035);  // 35 msec elapsed since last sample
  analogWrite(PIN_OUTPUT, Output);
}
```
