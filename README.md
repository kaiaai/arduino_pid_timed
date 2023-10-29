# Arduino PID library - with non-uniform time sampling support

This PID library is based on Brett Beauregard's
[Arduino PID library](https://github.com/br3ttb/Arduino-PID-Library)
adapted to work when time between samples is not constant.

## Example
```
my_pid = new PID(&measured_value, &knob_to_adjust, &target_value, kp, ki, kd, PID_DEFAULT_UPDATE_PERIOD, PID_MODE, DIRECT);
pid->SetOutputLimits(-1, 1);

pid->Compute(0.15); // 150msec elapsed since last sample
pid->Compute(0.18); // 180msec elapsed since last sample
pid->Compute(0.07); // 70msec elapsed since last sample

my_pid->enable(false); // freeze PID
my_pid->enable(true); // unfreeze PID
```
