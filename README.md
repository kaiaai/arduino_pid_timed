# Arduino PID library - with non-uniform time sampling support

This PID library is based on Brett Beauregard's
[Arduino PID library](https://github.com/br3ttb/Arduino-PID-Library)
adapted to work when time between samples is not constant.

## Example
```
#define PID_DEFAULT_UPDATE_PERIOD_SEC 0.01  // 10 msec

double Kp=2, Ki=5, Kd=1;
double Target_Value, Measured_Value, Controlled_by_PID;
Target_Value = 100;

PID pid(&Measured_Value, &Controled_by_PID, &Target_Value, Kp, Ki, Kd,
  PID_DEFAULT_UPDATE_PERIOD_SEC, DIRECT);
pid->SetOutputLimits(-1, 1);

Measured_Value = get_measured_value();
delay(150);
pid.Compute(0.15); // 150msec elapsed since last sample

Measured_Value = get_measured_value();
delay(180);
pid.Compute(0.18); // 180msec elapsed since last sample

Measured_Value = get_measured_value();
delay(70);
pid.Compute(0.07); // 70msec elapsed since last sample

pid.enable(false); // freeze PID

Measured_Value = get_measured_value();
delay(100);
pid.Compute(0.1); // No effect

pid.enable(true); // unfreeze PID
```
