# SET_ATTITUDE_TARGET changes — running log

Purpose: cross-session memory of what's been tried on `mavlink_client.send_attitude_target()`
and why, so the issue doesn't need to be re-explained from scratch each session. Append to
this file (don't rewrite history) whenever the attitude-target control scheme changes or a
bench/flight test reveals something new.

## Current state (2026-07-02)

**Symptom reported:** yaw was steering the rudder toward what looked like a compass-referenced
direction ("toward north") instead of a body-relative correction, and the elevator/pitch
response was also wrong relative to body axis while tracking.

**Root cause (confirmed against ArduPlane source, not guessed):** ArduPlane's GUIDED
`SET_ATTITUDE_TARGET` handler treats the 3 axes of the quaternion completely differently:

- `ArduPlane/GCS_MAVLink_Plane.cpp` converts the incoming quaternion to Euler and stores it
  verbatim: `forced_rpy_cd.x/y/z = degrees(q.get_euler_roll/pitch/yaw()) * 100`.
- `ArduPlane/mode_guided.cpp` feeds `forced_rpy_cd.x` → `nav_roll_cd` and `forced_rpy_cd.y` →
  `nav_pitch_cd`, constrained to the normal roll/pitch limits. These are **absolute angle
  targets** consumed by Plane's own attitude PID, which computes its own error against the
  current attitude. Composing camera error onto current attitude is *correct* here.
- `ArduPlane/Attitude.cpp`'s `calc_nav_yaw_coordinated()` instead does
  `commanded_rudder = forced_rpy_cd.z` **directly** — no PID, no reference to current heading.
  It's an open-loop rudder deflection command, not an attitude target.

So the previous code (`q = q_current * q_delta` composing roll/pitch/yaw together) sent an
absolute compass heading (in centidegrees) straight into `commanded_rudder`, which saturates
the rudder based on the aircraft's actual heading rather than the camera tracking error —
this is the "steers toward north" symptom. It also let yaw error leak into the *extracted*
pitch/roll (Euler decomposition of a composed quaternion couples axes), which is why the
elevator looked wrong too.

**Fix applied** (`mavlink_client.py::send_attitude_target`):
- Compose **only roll/pitch** error onto the FC's current attitude
  (`q_current * QuaternionBase([roll, pitch, 0.0])`), extract `roll_target`/`pitch_target`
  from that — same "hold current attitude" logic as before, but yaw no longer pollutes it.
- Send **yaw raw** (just the clamped camera yaw error, in radians) as the yaw component of
  the final quaternion — never composed with current heading — since the firmware wants a
  small open-loop rudder deflection there, not an absolute heading.
- Final quaternion built via `QuaternionBase([roll_target, pitch_target, yaw])`.

**Bench-verified 2026-07-02** (disarmed, GUIDED, synthetic pitch/yaw error vectors sent
directly — no camera needed — `SERVO_OUTPUT_RAW` read back; script was
`bench_test_attitude.py`, not checked into the repo, ask if you want it recreated):

| Test | Rudder servo1/servo3 (µs) | Elevon servo2/servo4 (µs) |
|---|---|---|
| neutral | 1500 / 1500 | 1500 / 1500 |
| yaw +15° | 1367 / 1633 | 1500 / 1500 (unchanged) |
| yaw -15° | 1633 / 1367 | 1500 / 1500 (unchanged) |
| pitch +15° | 1500 / 1500 (unchanged) | 1588 / 1411 |
| pitch -15° | 1500 / 1500 (unchanged) | 1411 / 1588 |
| pitch+15 & yaw+15 | 1367 / 1633 (matches yaw+15 alone) | 1588 / 1411 (matches pitch+15 alone) |
| pitch+15 & yaw-15 | 1633 / 1367 (matches yaw-15 alone) | 1588.5 / 1411 (matches pitch+15 alone) |

Aircraft's actual heading during the whole test was a fixed -40.3° (not physically rotated).
`yaw_err=0` still produced dead-center rudder (1500/1500) despite that nonzero heading —
under the old buggy full-composition, a -40.3° heading baked into the quaternion would have
shown up as a nonzero rudder output even at neutral, so this is direct on-hardware
confirmation the heading-independence fix works. Combined pitch+yaw vectors produced
byte-identical (or noise-level) outputs to the single-axis tests, confirming the cross-axis
coupling bug is also gone. FC came up armed+GUIDED before this test started (pre-existing
state, unrelated to this change) — disarmed via the existing `disarm()` path before sending
anything, left disarmed/MANUAL at the end.

**Not yet done:** flight test (this was bench-only, aircraft stationary — camera-driven
tracking loop via `tracker-so.py` with a real target hasn't been exercised against this fix).

## History (superseded attempts, kept for context)

1. **Body-rate `SET_ATTITUDE_TARGET`** (`type_mask=0b10000000`): disproved on the bench —
   ArduPlane's GUIDED handler doesn't act on body rate fields for a plane (that's Copter-only
   behavior). All 4 servo channels stayed frozen for ~10s of varying rate commands.
2. **Non-composed absolute quaternion** (send camera error directly, no current-attitude
   read): worked in an initial disarmed bench test, but a follow-up hand-tilt test reproduced
   the elevator pushing nose-down even with the camera centered, because "centered" was being
   sent as literal level flight instead of "hold current attitude." Abandoned same day
   (commit `0565489`, 2026-07-01).
3. **Full composition, all 3 axes together** (`q = q_current * q_delta` where
   `q_delta = QuaternionBase([roll, pitch, yaw])`): fixed the pitch-hold regression from #2,
   but is the version that caused today's bug — composing yaw onto current heading breaks
   because ArduPlane uses extracted yaw as a raw rudder command, not an attitude target (see
   root cause above). This also has a known latency artifact: the composed target is only as
   fresh as the last received ATTITUDE message, so a *fast* manual tilt on the bench can
   outrun it, producing a transient dip that decays as fresh telemetry catches up. Mostly
   mitigated by requesting a 20 Hz ATTITUDE stream (`_request_attitude_stream()`) and fixing
   a `tracker-so.py` main-loop bug that was flooding the serial link (frame_ready wait gate
   never re-armed after the first frame → loop free-spun at 100-500 Hz instead of actual
   camera frame rate).
4. **Current: split composition** (this file's "Current state" section above) — roll/pitch
   composed onto current attitude, yaw sent raw/uncomposed. Not yet bench-verified.

## Airframe context

X-UAV Talon flying wing, ArduPilot/ArduPlane V4.6.3. Elevons on SERVO2/4 (mixed
roll+pitch), differential-spoiler rudder-equivalent on SERVO1/3 (yaw). Not a copter — no
independent body-rate yaw actuation the way a multirotor has; turns are coordinated via
roll/bank, and per the root-cause finding above, GUIDED-mode yaw is actually an open-loop
rudder deflection command, not a heading-hold target.
