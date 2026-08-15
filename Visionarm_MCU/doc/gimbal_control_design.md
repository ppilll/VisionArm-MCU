# V6 Step I/J - Gimbal Control Design

## Scope

This stage converts the validated latest `CONTROL_UPDATE` image error into a
bounded incremental servo position command while preserving the Step-H safety
gate.

No PID, integral term, derivative term, estimator, or camera closed-loop tuning
is introduced here.

## Input contract

Only a `CONTROL_UPDATE` that has already passed V5 protocol/business validation
can reach the controller.

Synthetic Step-K coordinate contract:

- `error_x_q15 > 0`: target is right of image center.
- `error_x_q15 < 0`: target is left of image center.
- `error_y_q15 > 0`: target is below image center.
- `error_y_q15 < 0`: target is above image center.

The real V4 sign convention remains subject to the later limited live-direction
test. Step K does not claim that V4 has already been verified against this
contract.

## Measured actuator directions

- Pan PWM increase -> camera LEFT.
- Pan PWM decrease -> camera RIGHT.
- Tilt PWM increase -> camera DOWN.
- Tilt PWM decrease -> camera UP.

Therefore:

- Pan synthetic X input is inverted.
- Tilt synthetic Y input is not inverted.

## Frozen safe ranges

- Pan: 1000 / 1500 / 2000 us.
- Tilt: 1200 / 1500 / 1600 us.

## Controller

Each 20 ms control cycle:

1. Read latest validated Q15 error.
2. Apply axis sign/inversion.
3. Apply dead-zone (`|error| <= 2048`).
4. Convert normalized error into incremental pulse request.
5. Guarantee at least 1 us/cycle outside the configured dead-zone so integer
   truncation does not create another hidden dead-zone.
6. Clamp delta to +/-2 us/cycle.
7. Integrate delta into the axis target.
8. Clamp target to installed software-safe range.
9. Pass the absolute target to ActuatorDriver.

Full-scale proportional request before slew limiting is 4 us/cycle.
The physical target slew is limited to 2 us/cycle = 100 us/s.

## Behavior at zero error

Zero/dead-zone error holds the current commanded camera position. It does not
return the gimbal to the global center.

## Behavior at software limit

An outward command remains blocked at the configured min/max. A reverse command
is allowed immediately so the axis can leave the limit.

## Safety interaction

Any Step-H safety rejection:

- disables physical PWM through ActuatorDriver;
- resets controller state to calibrated center;
- requires fresh control according to GimbalSafety before re-enable.

On a fresh safety lease the actuator is enabled at center and the first
controller step is bounded by the normal per-cycle slew limit.

## STATUS V1 compatibility

The 52-byte V1 STATUS layout is unchanged.

The historical fields:

- `pan_stub_q15`
- `tilt_stub_q15`

now report normalized **applied actuator position command**:

- safe min -> -32767
- center -> 0
- safe max -> +32767

Because Tilt has asymmetric calibrated travel around center, normalization uses
its own lower and upper half-range independently.

## Why no PID yet

S20F already closes its internal shaft-position loop. The STM32 has no external
axis encoder. V6 therefore implements safe camera-command integration and
bounded motion. Camera-loop response tuning, overshoot, settling, filtering and
PID evaluation belong to V7.
