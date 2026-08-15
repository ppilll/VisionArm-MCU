# V6 Step I/J/K Acceptance Plan

## Step I - AxisController

PASS when:

- startup controller self-test passes;
- dead-zone works;
- synthetic sign/inversion is deterministic;
- proportional incremental command works;
- per-cycle slew <= 2 us;
- Pan stays within 1000..2000 us;
- Tilt stays within 1200..1600 us;
- outward motion is blocked at limit;
- reverse motion can leave limit.

## Step J - GimbalTask

PASS when:

- former gimbal_stub task is no longer created;
- `GimbalTask_Create()` is the application consumer;
- static allocation only;
- priority remains 2;
- period remains 20 ms / 50 Hz;
- uses `xTaskDelayUntil()`;
- V5 ProtocolRx priority remains 5;
- V5 ProtocolTx priority remains 4;
- SafetyGate remains outside/above controller output;
- unsafe condition disables ActuatorDriver;
- valid fresh control drives GimbalController -> ActuatorDriver;
- STATUS V1 layout is unchanged.

## Step K - Synthetic Control

Use the RK3588 contract in `rk3588_synthetic_control_contract.md`.

Required cases:

- zero;
- inside dead-zone;
- positive X;
- negative X;
- positive Y;
- negative Y;
- full-scale;
- rapidly changing signs;
- stale control.

Capture at least one logic-analyzer trace proving the 2 us/cycle maximum PWM
command change and one trace proving stale-control output disable.
