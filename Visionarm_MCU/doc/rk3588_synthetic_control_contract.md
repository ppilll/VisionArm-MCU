# RK3588 Synthetic Control Contract for V6 Step K

This document is the implementation contract for the RK3588-side synthetic
communication tester. It uses the existing V5 protocol without any new message
type or wire-format change.

## Session rules

1. Use one nonzero `sender_boot_id` for the whole test session.
2. `wire_sequence` is global across every outbound message type and must be
   monotonically increasing.
3. Start with `HELLO` and wait for accepted `HELLO_ACK`.
4. Send `HEARTBEAT` every 100 ms. Each heartbeat should receive STATUS.
5. Send valid `CONTROL_UPDATE` every 20 ms during active test phases.
6. Do not exceed the MCU-advertised max control rate (100 Hz in current V5).

## HELLO payload

Recommended deterministic values:

- `device_role = 1`
- `minimum_protocol_version = 1`
- `maximum_protocol_version = 1`
- `max_payload = 128`
- `max_control_rate_hz = 50`
- `capability_bits = 0`
- software version fields may identify the RK test utility

## Valid CONTROL_UPDATE payload

- `target_state = 2` (`DETECTED`)
- `control_flags = 0x01` (`VALID`)
- `capture_age_at_tx_ms = 0`
- `confidence_u16 = 65535`
- `dx_px = 0` and `dy_px = 0` for Step K; MCU controller does not use them
- increment source frame/session fields normally; they are diagnostic here

Only `error_x_q15` and `error_y_q15` drive Step-I/J controller behavior.

## Synthetic sign contract

- `error_x_q15 > 0` => target RIGHT => Pan PWM decreases => camera RIGHT.
- `error_x_q15 < 0` => target LEFT  => Pan PWM increases => camera LEFT.
- `error_y_q15 > 0` => target BELOW => Tilt PWM increases => camera DOWN.
- `error_y_q15 < 0` => target ABOVE => Tilt PWM decreases => camera UP.

This is a synthetic-test contract. The real V4 sign is verified later.

## Controller constants relevant to expected results

- Gimbal task: 20 ms / 50 Hz.
- Dead-zone: `|error| <= 2048`.
- Full-scale proportional request: 4 us/cycle.
- Physical slew clamp: max 2 us/cycle.
- Pan safe range: 1000..2000 us; center 1500 us.
- Tilt safe range: 1200..1600 us; center 1500 us.

Typical incremental behavior:

- `|error| = 1024`: 0 us/cycle (dead-zone).
- `|error| = 8192`: about 1 us/cycle.
- `|error| = 16384`: about 2 us/cycle.
- full-scale: request about 4 us/cycle, applied max 2 us/cycle.

## STATUS interpretation

STATUS remains 52 bytes.

- `pan_stub_q15` now means normalized applied Pan command.
- `tilt_stub_q15` now means normalized applied Tilt command.

Mapping per calibrated axis half-range:

- safe min -> -32767
- center -> 0
- safe max -> +32767

These are actuator command diagnostics, not camera error values.

## Required Step-K sequence

### K0 - Center and dead-zone

For 1000 ms send at 50 Hz:

`error_x_q15 = 0`, `error_y_q15 = 0`.

Expected:

- actuator enables at center;
- PA6 ~= 1500 us;
- PA7 ~= 1500 us;
- STATUS pan/tilt command ~= 0.

Then for 500 ms:

`error_x_q15 = +1024`, `error_y_q15 = -1024`.

Expected: no commanded movement.

### K1 - Pan positive synthetic error

For 600 ms:

`error_x_q15 = +8192`, `error_y_q15 = 0`.

Expected:

- Pan pulse decreases about 1 us per 20 ms controller cycle;
- camera moves RIGHT;
- Tilt remains approximately unchanged.

Then for 600 ms send `error_x_q15 = -8192` to move back toward center.

### K2 - Pan negative synthetic error

For 600 ms:

`error_x_q15 = -8192`, `error_y_q15 = 0`.

Expected camera LEFT. Follow with +8192 for 600 ms to return.

### K3 - Tilt positive synthetic error

For 600 ms:

`error_x_q15 = 0`, `error_y_q15 = +8192`.

Expected Tilt pulse increases and camera moves DOWN. Follow with -8192 for
600 ms to return.

### K4 - Tilt negative synthetic error

For 600 ms:

`error_x_q15 = 0`, `error_y_q15 = -8192`.

Expected camera UP. Follow with +8192 for 600 ms to return.

### K5 - Full-scale/slew test

For 200 ms:

`error_x_q15 = +32767`, `error_y_q15 = +32767`.

Then 200 ms:

`error_x_q15 = -32768`, `error_y_q15 = -32768`.

Expected:

- no axis target changes by more than 2 us in one 20 ms controller cycle;
- no output leaves calibrated software-safe limits.

Use logic analyzer for the per-cycle delta evidence.

### K6 - Rapid sign-change test

For 2000 ms alternate every 100 ms:

A: `(+32767, +32767)`

B: `(-32768, -32768)`

Keep CONTROL_UPDATE at 50 Hz inside each 100 ms block.

Expected:

- small bounded oscillation around the current command;
- no pulse-width jump greater than 2 us per 20 ms controller cycle;
- no dense-edge PWM regression;
- no MCU reset or UART/parser error growth.

### K7 - Stale-control test

1. Send `(+8192, 0)` valid controls for 500 ms.
2. Keep HEARTBEAT every 100 ms.
3. Stop CONTROL_UPDATE completely for at least 350 ms.

Expected after the existing ~200 ms control freshness timeout plus task/polling
latency:

- STATUS `link_state` remains READY;
- STATUS `control_valid` becomes 0;
- PA6/PA7 PWM is disabled and pins go LOW.

Then send a new valid `(0,0)` CONTROL_UPDATE.

Expected:

- fresh generation permits re-enable;
- actuator re-enters through calibrated center;
- controller remains in dead-zone at center.

## Failure criteria

Fail Step K if any of the following occurs:

- positive X moves camera left;
- positive Y moves camera up;
- dead-zone input causes persistent movement;
- any one-cycle pulse target change exceeds 2 us;
- Pan command leaves 1000..2000 us;
- Tilt command leaves 1200..1600 us;
- stale control leaves PWM active;
- old control resumes without a fresh generation after safety disable;
- UART/parser error counters increase unexpectedly;
- MCU resets, actuator jumps, or PWM develops dense edges.
