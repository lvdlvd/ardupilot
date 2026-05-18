# HDGALT Implementation Plan

A staged implementation plan for the `HDGALT` ArduPlane mode
specified in `ArduPlane/mode_hdgalt.md`. Each numbered section is a
self-contained step that produces a buildable, testable artifact.
Steps are ordered so that each builds on the previous; do not skip
ahead.

The plan is written to be executed by a coding assistant working in
the ardupilot tree on branch `hdgaltmode`. Read the design document
(`ArduPlane/mode_hdgalt.md`) before starting any step; it is the
authoritative specification. This plan is the *order of operations*,
not the spec.

---

## Working assumptions

- Branch: `hdgaltmode`, off `master`.
- Build target: SITL (`./waf configure --board sitl && ./waf plane`).
- Test harness: ArduPilot's own `Tools/autotest/` framework. The
  external `ardusitl` harness is irrelevant for upstream contribution;
  rely on the official tests.
- Style: follow ArduPlane conventions everywhere. Match existing
  mode files for formatting, naming, indentation, brace style.
  When in doubt, mimic `mode_cruise.cpp` and `mode_cruise.h` — they
  are the closest existing mode in spirit to HDGALT.
- Commit cadence: each numbered step is one commit (or a small
  series). The branch should remain buildable at every commit.

---

## Step 1: Define the MAVLink messages

**Status: DONE** — acceptance met and agreed. Submodule
`modules/mavlink@hdgaltmode` commit `2f5675fa`; parent commit
`7412a4a66a`. `HDGALT_COMMAND` = id 11061, `HDGALT_STATE` = id
11062, `PLANE_MODE_HDGALT` = 27, supporting enums added; `./waf
plane` builds and generates the headers.

Goal: nail down `HDGALT_COMMAND` and `HDGALT_STATE` as concrete
MAVLink message definitions. Nothing else in the implementation can
proceed without this contract.

### What to do

1. Create a new MAVLink dialect file. The conventional location for
   ArduPilot-specific extensions is `modules/mavlink/message_definitions/v1.0/ardupilotmega.xml`.
   Inspect that file to see how other ArduPilot-specific messages
   (e.g. `RANGEFINDER`, `AHRS3`, `MEMINFO`) are declared.
2. Add `HDGALT_COMMAND` and `HDGALT_STATE` message definitions per
   section 5 of the design doc. Pick currently-unused message IDs
   in the ardupilotmega range. Document each field with a one-line
   `<description>` matching the design doc.
3. Add `HDGALT` to the `MAV_MODE_FLAG` family if appropriate, or to
   whatever enum ArduPlane uses for its custom modes. Look at how
   `CRUISE` and `AUTO` are declared and follow the same pattern.
4. Regenerate the MAVLink C headers via the standard build step
   (the waf build will pick this up automatically, but verify by
   building once after the XML edit).

### Acceptance

- `./waf plane` builds successfully.
- The generated headers contain `mavlink_msg_hdgalt_command.h` and
  `mavlink_msg_hdgalt_state.h`.
- Manually inspect those generated headers: the field types and
  ordering match the design doc.

### Notes

- Field widths matter for MAVLink: `float` is 4 bytes, `uint32` is
  4 bytes, etc. The design doc's pseudo-code uses these types; the
  XML must match.
- `flags` is a `uint16` bitfield. Define the bit values as
  `<entry>` elements in an enum, e.g. `HDGALT_COMMAND_FLAG_HEADING = 1`,
  `HDGALT_COMMAND_FLAG_TURN_RATE = 2`, etc.
- The `warnings` field in `HDGALT_STATE` is similarly a `uint16`
  bitfield with `HDGALT_WARNING_*` entries.
- Document the time field semantics clearly: `start_time_boot_ms`
  is "AP-local clock, milliseconds since boot," same time-base as
  `SYSTEM_TIME.time_boot_ms`.

### Do not yet

Do not write any C++ handlers for these messages. That comes in
step 6. This step is XML-only.

---

## Step 2: Stub mode in ArduPlane

**Status: DONE** — acceptance met and agreed (committed together
with this status note). `Mode::Number::HDGALT = 27`, inert
`ModeHdgAlt` class, factory + `Plane` member registered. SITL:
mode switches to HDGALT (HEARTBEAT `custom_mode = 27`), arming
refused ("HDGALT mode not armable"). Plan/design-doc deviations
recorded in design doc section 8.1: ArduPlane has no
`requires_GPS()`/`allows_arming()` virtuals — arming-disable is
done via `_pre_arm_checks() == false`; no `wscript` edit needed
(glob build); `-Wswitch` cases added in `GCS_Plane.cpp`,
`GCS_MAVLink_Plane.cpp`, `events.cpp`.

Goal: a new `HDGALT` mode that can be selected via the mode switch
in SITL, that does nothing yet but compiles and runs.

### What to do

1. Read `ArduPlane/mode.h`, `ArduPlane/mode.cpp`, and the
   `mode_cruise.h` / `mode_cruise.cpp` pair to understand the
   `Mode` class hierarchy and conventions.
2. Create `ArduPlane/mode_hdgalt.h` declaring a `ModeHdgAlt` class
   inheriting from `Mode`. Override:
   - `number()` returning `Number::HDGALT` (you'll add this enum
     value too).
   - `name()` returning `"HDGALT"`.
   - `name4()` returning `"HDGA"`.
   - `_enter()`, `_exit()`, `update()` (empty for now).
   - `requires_GPS()` returning `false`.
   - `allows_arming(AP_Arming::Method)` returning `false`.
   - `is_landing()` returning `false`.
3. Add `HDGALT` as a new enum value in `Mode::Number`. Pick the
   next unused integer.
4. Create `ArduPlane/mode_hdgalt.cpp` with skeleton implementations
   of all overridden methods. Most are no-ops; `update()` is
   empty.
5. Register the mode in ArduPlane's mode factory. This is typically
   in `mode.cpp`'s `Plane::mode_from_mode_num()` function — add a
   case for `Number::HDGALT` returning a pointer to the new mode
   instance.
6. Add the new mode instance as a member of `Plane` class. Find
   where other modes are declared (likely in `Plane.h` or
   `mode.cpp`) and add `ModeHdgAlt mode_hdgalt;` to the list.
7. Add `mode_hdgalt.cpp` to the build by listing it in
   `ArduPlane/wscript` (or whatever ArduPlane's build manifest is).

### Acceptance

- `./waf plane` builds successfully.
- Launch SITL: `Tools/autotest/sim_vehicle.py -v ArduPlane --console --map`.
- At the MAVProxy prompt, type `mode HDGALT`. The mode should
  switch and `mode` should report `HDGALT`. The plane will not
  fly correctly (the mode does nothing), but the mode should be
  selectable and not crash.
- `arm throttle` should be refused with a sensible message
  (because `allows_arming() = false`).

### Notes

- ArduPlane has many existing modes; expect the stub to require
  edits in 4-6 different files. The mode-registration pattern is
  not pretty but is consistent across modes.
- Some ArduPlane modes use `init()` instead of `_enter()`. Check
  the base class current API. Use whatever the existing modes use.

### Do not yet

Do not implement any control logic. The mode is intentionally inert.

---

## Step 3: Lateral controller — heading hold

**Status: DONE** — acceptance met and agreed (committed with this
note). Heading→bank uses `AC_PID` mirroring `g2.guidedHeading`
(design doc §8.1/§8.3 updated; the legacy `PID` class is unused in
modern Plane). Params: `HDGALT_BANK_MAX`, `HDGALT_STALL_MGN`,
`HDGALT_ASPD_MIN`, `HDGALT_HD_*` (names abbreviated for the
AP_Param 16-char limit; §4 updated). Added `friend class
ModeHdgAlt;` to `Plane`. Vertical is the CRUISE-style placeholder
(`set_target_altitude_current` + `update_fbwb_speed_height`),
replaced in step 4. The entire mode is now feature-gated behind
`MODE_HDGALT_ENABLED` (default 1) with a `build_options.py` entry;
both the enabled and `--define MODE_HDGALT_ENABLED=0` builds
compile. SITL: engaged at ~90 m, heading held within ~4° over 30 s,
bank within `HDGALT_BANK_MAX`.

Goal: a working heading PID + bank limit + stall-margin clip, such
that engaging HDGALT in SITL holds the heading at engagement.

### What to do

1. In `mode_hdgalt.h`, declare:
   - Private member: `float heading_cmd_deg` — the heading setpoint.
   - Private member: `AC_PID heading_pid` — the PID controller. (Or
     whichever PID class ArduPlane uses; check what `mode_cruise.cpp`
     uses for similar purposes.)
   - Private method: `float compute_bank_command()`.
2. In `_enter()`: snapshot the current heading from `ahrs.yaw`
   (converted from radians to degrees and wrapped to 0–360) into
   `heading_cmd_deg`. Reset the PID's integral state.
3. In `update()`:
   - Compute heading error: `wrap_180(heading_cmd_deg - current_heading_deg)`.
   - Run the PID on heading error to get a bank command.
   - Clip by bank limit and stall-margin limit (see section 3.3 of
     the design doc).
   - Pass the clipped bank command to the existing roll attitude
     controller. The call pattern depends on ArduPlane internals;
     look at how `mode_cruise.cpp` commands roll for the right API.
   - For now, vertical: just call whatever existing function `mode_cruise`
     calls to hold pitch level. This is a placeholder until step 4.
4. Add parameters for the lateral controller:
   - `HDGALT_BANK_MAX` (float, default 25.0, units degrees).
   - `HDGALT_STALL_MARGIN` (float, default 1.3, no units).
   - `HDGALT_AIRSPEED_MIN` (float, default value airframe-specific
     — set 10.0 m/s as a placeholder, with a comment noting this
     should be tuned).
   - Heading PID gains: typically a struct of params, follow how
     other modes declare their PID params. ArduPlane convention
     uses `AP_PARAM_FRAME_PLANE` for plane-only params.
5. Implement `compute_bank_command()` per the design doc's section
   3.3. The math:
   ```
   bank_request = PID(heading_error)
   bank_max = HDGALT_BANK_MAX
   ratio = HDGALT_AIRSPEED_MIN * HDGALT_STALL_MARGIN / current_airspeed
   if ratio >= 1.0: bank_stall_max = 0
   else: bank_stall_max = degrees(acos(ratio * ratio))
   bank_limit = min(bank_max, bank_stall_max)
   return constrain(bank_request, -bank_limit, +bank_limit)
   ```

### Acceptance

- `./waf plane` builds.
- In SITL, take off in MANUAL, fly to ~100m altitude and trim for
  cruise, then `mode HDGALT`. The plane should hold its heading.
- Push the simulated stick to one side: HDGALT should *not* respond
  to stick input yet (that's step 8). The plane will fight you.
  This is fine for now; just verify heading is maintained when no
  stick input is given.
- Inspect the BIN log: roll attitude commands from the PID should
  appear in `ATT` or similar message; the bank should stay below
  `HDGALT_BANK_MAX`.

### Notes

- Heading from `ahrs.yaw` is in radians, range `[-π, π]`. Convert
  to degrees and to the 0–360 range conventionally used for
  magnetic headings.
- The wrap function for heading error must handle the wrap-around:
  going from 350° to 10° is a +20° turn, not -340°. Use the
  existing ArduPilot utility `wrap_180()` from `AP_Math/AP_Math.h`.
- PID tuning: start with conservative gains. `kP = 0.5`, `kI = 0.05`,
  `kD = 0.0` is a reasonable starting point for heading→bank. Tune
  in SITL once it flies. The values will be airframe-dependent.

### Do not yet

Do not handle MAVLink commands. The heading remains whatever was
snapshotted at engagement. Step 6 adds command handling.

---

## Step 4: Vertical passthrough to TECS

**Status: DONE** — acceptance met and agreed (committed with this
note). Vertical now holds a commanded MSL altitude via TECS:
`_enter` snapshots `current_loc.alt`; each tick a slewed
`altitude_target_m` (capped at `HDGALT_CLIMB_RT`) is written to
`plane.target_altitude.amsl_cm`, `TECS set_pitch_max(HDGALT_PITCH_MAX)`
applied, then `calc_nav_pitch`+`calc_throttle` (LOITER pattern).
The climb-rate cap is enforced by setpoint slewing (ArduPlane has
no per-call TECS climb-rate API; design §3.4/§8.2 updated). New
params `HDGALT_CLIMB_RT` (2.0), `HDGALT_PITCH_MAX` (10). Both
`MODE_HDGALT_ENABLED` builds compile. SITL: over 60 s after
engagement, altitude held to 1.5 m and heading to 4.4 deg.

Goal: pass HDGALT's altitude setpoint to TECS so the plane holds
altitude as well as heading.

### What to do

1. In `mode_hdgalt.h`, add:
   - Private member: `float altitude_cmd_m` — the altitude setpoint.
   - Private member: `float climb_rate_cmd_mps` — the climb rate cap.
2. In `_enter()`: snapshot the current altitude (use baro altitude,
   `AP::baro().get_altitude()`) into `altitude_cmd_m`. Set
   `climb_rate_cmd_mps` from `HDGALT_CLIMB_RATE_DEFAULT`.
3. In `update()`, replace the placeholder vertical call with TECS
   commands. Look at `mode_cruise.cpp` to see how it interacts with
   TECS — typically through `plane.calc_throttle()` and
   `plane.calc_nav_pitch()` or via direct TECS API calls.
4. Set TECS targets:
   - Altitude: `tecs->set_altitude_target_m(altitude_cmd_m)` (verify
     the exact API name; ArduPlane's TECS has historically named
     these things variously).
   - Climb rate cap: pass `climb_rate_cmd_mps` to whatever TECS
     method controls maximum climb rate.
5. Add parameters:
   - `HDGALT_CLIMB_RATE_DEFAULT` (float, default 2.0, units m/s).
   - `HDGALT_PITCH_MAX` (float, default 10.0, units degrees) —
     passed to TECS as pitch limit.

### Acceptance

- Build and SITL test as in step 3.
- After takeoff, climbing to cruise altitude in MANUAL, then engaging
  HDGALT: the plane should hold both heading and altitude.
- In MAVProxy, observe altitude via `status`: should remain within
  ±5m of the engagement altitude over a 1-minute hold (depending
  on TECS tuning).

### Notes

- TECS is well-tuned in stock ArduPlane. You should not need to
  modify its gains; HDGALT just feeds it targets.
- If altitude drifts more than expected, the likely cause is that
  the wrong TECS API is being called. Diff against `mode_cruise`
  carefully; CRUISE holds altitude similarly and is a good
  reference.
- Climb-failure detection (the latched failsafe per section 6.4)
  is **not** in this step. That comes in step 5 once we know how
  TECS exposes saturation.

---

## Step 5: Climb-failure detection

**Status: DONE** — acceptance met and agreed (committed with this
note). Added a minimal library-wide accessor
`AP_TECS::height_demand_limited()` exposing TECS's existing
height-demand freeze (it had no public saturation getter). HDGALT
latches the failsafe after `HDGALT_CLIMB_FT` s of that being true
with altitude error (to commanded alt) outside a 5 m deadband:
clips `altitude_cmd_m`/target to current, emits one STATUSTEXT.
Latch clears only on a new HDGALT_COMMAND (hook documented; wired
in step 6). New param `HDGALT_CLIMB_FT` (10 s). Validated with a
TEMP +300 m instrumentation in `_enter` (per this step's notes, as
no command path exists yet) — latched once with STATUSTEXT after
the timeout, no repeat; instrumentation reverted before commit.
Enabled and `MODE_HDGALT_ENABLED=0` builds compile. Design doc
§3.4/§6.4/§8.2 updated. End-to-end re-validated at step 6 via the
real MAVLink path: an unreachable HDGALT_COMMAND latches the
failsafe; a new command clears it; it can re-latch (proving the
clear is real, not just quiet).

Goal: detect when TECS can't reach the commanded altitude and latch
the failsafe.

### What to do

1. Examine TECS's interface for saturation/achievability signals.
   Likely candidates:
   - A method like `tecs->is_underspeed()` or similar that
     indicates the airplane is at its limits.
   - A way to read the demanded throttle and check if it's at max.
   - Altitude error history.
2. Add private state to `ModeHdgAlt`:
   - `bool climb_failed_latched`
   - `uint32_t climb_fail_start_ms` — when the saturation condition
     started.
3. In `update()`:
   - Compute altitude error: `altitude_cmd_m - current_altitude`.
   - If `abs(error) > deadband` (e.g. 5m) AND TECS is saturated
     trying to reduce the error:
     - If `climb_fail_start_ms` is 0 (not currently in the
       saturated state): set it to `millis()`.
     - Else if `millis() - climb_fail_start_ms > HDGALT_CLIMB_FAIL_TIMEOUT * 1000`:
       latch the failsafe.
   - Else (error within deadband, or TECS not saturated): reset
     `climb_fail_start_ms = 0`.
4. When the failsafe latches:
   - Clip `altitude_cmd_m` to current altitude.
   - Set the `HDGALT_WARNING_CLIMB_UNACHIEVABLE` bit (you'll have a
     `warnings` member by step 7; for now, use a bool and emit
     STATUSTEXT now).
   - Emit one STATUSTEXT at `MAV_SEVERITY_WARNING`: "HDGALT: climb
     unachievable, holding altitude".
5. When a new HDGALT_COMMAND arrives (step 6) and updates
   `altitude_cmd_m`, clear `climb_failed_latched` and
   `climb_fail_start_ms`. (Add a TODO comment for now; wire it up
   in step 6.)
6. Add parameter:
   - `HDGALT_CLIMB_FAIL_TIMEOUT` (float, default 10.0, units s).

### Acceptance

- Build and SITL test.
- Reduce throttle in SITL (via MAVProxy: `param set TRIM_THROTTLE 30`
  or similar) so the airplane can't climb. Engage HDGALT and set a
  high altitude target manually (you can do this temporarily by
  editing `altitude_cmd_m` in `_enter()` for testing).
- Observe: after `HDGALT_CLIMB_FAIL_TIMEOUT` seconds, the
  STATUSTEXT appears and altitude is clipped.

### Notes

- TECS's exact API for "I'm saturated" varies by ArduPlane version.
  Read the current `AP_TECS.h` and `AP_TECS.cpp` carefully. If no
  clean API exists, you may need to add one — that's a small
  upstream-friendly change.
- Be careful with the deadband: too small and normal altitude
  oscillation triggers the failsafe; too large and real failures
  go undetected. 5m is a reasonable starting point for a fixed-wing.

---

## Step 6: MAVLink command handling

**Status: DONE** — acceptance met and agreed (committed with this
note). `GCS_MAVLINK_Plane::handle_message` routes
`HDGALT_COMMAND` (guarded) → decodes → calls
`ModeHdgAlt::handle_hdgalt_command(...)` with scalars (no MAVLink
types in mode.h — signature change from the design sketch, §8.1/§8.4
updated). Immediate / past / future (single-slot queue, AP-clock
gated) handling; per-axis flag honouring; new param
`HDGALT_TURN_RATE` (3.0) which also caps bank via
omega=g*tan(phi)/V. `apply_command` clears the step-5 latch.
Command dropped unless HDGALT is active.

Acceptance run **GPS-denied** (option A: GPS only for the
mode-independent arm/climb, then `SIM_GPS1_ENABLE=0` in flight
before HDGALT engage): HDGALT engaged with no fix; immediate
heading +90 tracked (6.3°), immediate altitude +60 m tracked
(6.1 m), queued +8 s command did not act early (0.2°) then
converged (6.9°). Step 5↔6: unreachable command latched the
failsafe, a new command cleared it, and it re-latched. Both
`MODE_HDGALT_ENABLED` builds compile.

NOTE (follow-up, post-plan): the autotest scaffold still needs GPS
for the *takeoff/arm* (ArduPlane requires home + EKF-yaw to arm,
independent of mode/GPS). To be addressed after the plan; HDGALT
itself uses no GPS, as demonstrated.

Goal: process incoming `HDGALT_COMMAND` messages, update setpoints,
handle the single-slot future queue.

### What to do

1. In `GCS_Mavlink_Plane.cpp` (or wherever incoming MAVLink messages
   are routed), add a case for `MAVLINK_MSG_ID_HDGALT_COMMAND`.
   Route to `plane.mode_hdgalt.handle_HDGALT_COMMAND(msg)`.
2. Implement `ModeHdgAlt::handle_HDGALT_COMMAND(const mavlink_message_t &msg)`:
   - Decode the message into a `mavlink_hdgalt_command_t` struct.
   - If `start_time_boot_ms == 0`: apply immediately. Update
     `heading_cmd_deg`, `altitude_cmd_m`, etc. from the message
     fields, respecting the flags bitfield.
   - If `start_time_boot_ms > current AP_HAL::millis()`: store the
     command in a pending-command slot. Any previously-pending
     command is overwritten.
   - If `start_time_boot_ms` is in the past: apply immediately and
     log a warning that the command was late.
3. Add private state:
   - A pending-command struct with the same fields as the message
     plus a `valid` flag.
4. In `update()`:
   - At the top of the tick, check the pending command. If valid
     and its start time has been reached: apply it, clear the
     valid flag.
5. When `heading_cmd_deg`, `altitude_cmd_m`, or `climb_rate_cmd_mps`
   is updated by a command: reset any active failsafe latches
   (from step 5).
6. Respect the flags bitfield: if `HDGALT_COMMAND_FLAG_HEADING` is
   not set in the message's `flags`, leave `heading_cmd_deg`
   unchanged. Same for the other axes.

### Acceptance

- Build.
- In SITL, take off and engage HDGALT manually.
- From MAVProxy, send a HDGALT_COMMAND with the heading set to
  current+90 and immediate execution:
  ```
  long HDGALT_COMMAND <fields...>
  ```
  (MAVProxy syntax for sending arbitrary commands.)
- Observe: the plane turns 90° to the new heading.
- Send another with `start_time_boot_ms = current + 10000` (10
  seconds in the future): the command should queue, and after 10
  seconds the new turn should begin.

### Notes

- The MAVLink command-sending from MAVProxy is awkward. You may
  find it easier to write a tiny Python script using `pymavlink`
  to send well-formed HDGALT_COMMANDs for testing. Don't formalize
  this; it's just a test scaffold.
- Logging: emit a brief `AP_Logger` entry every time a HDGALT_COMMAND
  arrives. This shows up in the BIN log for post-flight analysis.

---

## Step 7: HDGALT_STATE emission

**Status: DONE** — acceptance met and agreed (committed with this
note). `ModeHdgAlt::send_hdgalt_state()` fills `mavlink_hdgalt_state_t`
and broadcasts via `gcs().send_to_active_channels()` (AP_Button /
AP_Avoidance pattern — no library-side SRx/ap_message entry needed),
rate-limited to 1 Hz from `update()`. `_actual` fields:
`ahrs.get_yaw_rate_earth()`, `barometer.get_climb_rate()` (both
GPS-free); `capture_state` from 5°/5 m thresholds; warnings =
CLIMB_UNACHIEVABLE / TURN_DERATED / AIRSPEED_LOW. SITL: streamed at
1.00 s intervals with sane fields; commanding turn_rate 30 deg/s
(unachievable) gave turn_rate_actual ~11.9 < commanded 30 with
HDGALT_WARNING_TURN_DERATED set. Both MODE_HDGALT_ENABLED builds
compile. Design doc §5.2/§8.4 updated.

Goal: periodically emit `HDGALT_STATE` so the FD (or a GCS) can see
what the AP is doing.

### What to do

1. Read how other modes emit periodic MAVLink messages. The pattern
   is usually a `stream_*` function in `GCS_Mavlink_Plane.cpp` or
   a periodic call from `update()`.
2. Implement `ModeHdgAlt::send_state(mavlink_channel_t chan)`:
   - Fill a `mavlink_hdgalt_state_t` struct from current mode state.
   - `_target` fields are the setpoints, `_current` fields are
     measured values, `_actual` fields are what the AP is doing
     (e.g., `turn_rate_actual` is current yaw rate; `climb_rate_actual`
     is current vertical speed from baro derivative).
   - `capture_state` reflects whether the plane is in hold (errors
     small) or capture (errors large) on each axis.
   - `warnings` is the failsafe bits.
3. Schedule the state message at 1 Hz minimum. Look at how
   `GLOBAL_POSITION_INT` is scheduled and follow the same pattern.

### Acceptance

- Build.
- In SITL with HDGALT engaged, send HDGALT_COMMANDs and observe
  HDGALT_STATE messages flowing back.
- Specifically test the `_actual` vs `_target` divergence cases:
  command a turn rate above what bank/stall limits allow, and
  verify that `turn_rate_actual < turn_rate_commanded` and the
  `HDGALT_WARNING_TURN_DERATED` bit is set.

### Notes

- ArduPlane has facilities for scheduling MAVLink streams at
  configurable rates. Default 1 Hz; tunable via `SR0_*` parameters.

---

## Step 8: Pilot override

**Status: DONE** — acceptance met and agreed (committed with this
note). `check_pilot_override()` runs first in `update()`; any
roll/pitch `norm_input` beyond `HDGALT_PILOT_THR`, or a throttle
**movement from its engage-captured position** beyond the
threshold, disengages immediately to `HDGALT_DISENG` (default
FBWA) via `set_mode(..., RC_COMMAND)` with a STATUSTEXT. Rudder is
not a trigger. Whole check gated on `rc().has_valid_input()`.
New params `HDGALT_PILOT_THR` (0.10), `HDGALT_DISENG` (5).

DEVIATION (design §3.5): the literal "throttle deviates from
TECS-commanded" rule disengages the instant HDGALT engages (the
throttle lever is not spring-centred and rarely matches TECS at
engage — confirmed in SITL). Implemented the design *intent*
("deliberate push or pull") as movement from the throttle position
captured at engage. Design doc §3.5 updated; also fixed §4
`HDGALT_DISENG` (the 18-char `HDGALT_DISENG_MODE` exceeds the
AP_Param 16-char limit) and normalised all old param names in the
doc.

GAP: design §6.5 (disengage when airspeed < `HDGALT_ASPD_MIN`) is
specified but is not a numbered plan step; raised with the user.

SITL: no-RC → stays engaged; neutral override → no false trigger;
roll nudge → disengaged to FBWA; re-engage works. Both
`MODE_HDGALT_ENABLED` builds compile.

Goal: detect any meaningful pilot stick input and disengage HDGALT,
switching to FBWA (or `HDGALT_DISENGAGE_MODE`).

### What to do

1. Add parameters:
   - `HDGALT_PILOT_THRESHOLD` (float, default 0.10, no units —
     fraction of stick throw).
   - `HDGALT_DISENGAGE_MODE` (int, default to the int value for
     FBWA mode).
2. In `update()`, at the top:
   - Read pilot roll input from the appropriate RC channel
     (`channel_roll->get_control_in()` or similar — examine
     `mode_fbwa.cpp` for the right call).
   - Read pilot pitch input.
   - Read pilot throttle position. Compare to TECS-commanded
     throttle: `abs(pilot_throttle - tecs_commanded_throttle) > threshold`.
   - If *any* of these exceed `HDGALT_PILOT_THRESHOLD` (expressed
     as a fraction of stick travel, so e.g. 100 in raw units of
     ±500 if that's the convention):
     - Log a STATUSTEXT: "HDGALT: pilot override".
     - Call `plane.set_mode(...)` to switch to
       `HDGALT_DISENGAGE_MODE`.
     - Return from `update()` without doing further control work.
3. Yaw (rudder) is **not** an override trigger. Document this with
   a comment.
4. The deadband applies *each tick* — if the stick is in the
   deadband, no disengage. If it leaves the deadband for one tick,
   disengage immediately. No debounce.

### Acceptance

- Build.
- In SITL, take off, engage HDGALT, then nudge roll stick slightly.
- The mode should switch to FBWA on the first tick where stick
  position exceeds threshold.
- Re-engaging HDGALT via `mode HDGALT` should work and snapshot the
  new state.

### Notes

- Threshold in raw stick units depends on RC channel conventions.
  If the channel's full deflection is ±500, then a 10% threshold
  is 50 units. Look at how `mode_fbwa` handles raw stick input
  for the conversion.
- The throttle comparison is the tricky one. TECS commands
  throttle as a value 0–100 (or 0–1). The pilot's throttle stick is
  in raw RC units. The comparison must be against equivalent units;
  normalize to fraction of full range on both sides.

---

## Step 9: Integration test in `Tools/autotest/`

**Status: NOT STARTED.**

Goal: an automated test that verifies HDGALT works end-to-end, run
as part of ArduPilot's standard test suite.

### What to do

1. Examine `Tools/autotest/arduplane.py` to understand how plane
   mode tests are structured. Each test is typically a method on
   the `AutoTestPlane` class.
2. Add a method `test_hdgalt(self)`:
   - Take off in MANUAL, climb to 100m, trim for cruise.
   - Switch to HDGALT.
   - Verify that heading and altitude are held within tolerance
     for 10 seconds (use the autopilot's reported `VFR_HUD` data
     against the snapshot at engagement).
   - Send a HDGALT_COMMAND with a new heading 90° to the right.
     Verify that the plane turns and stabilizes on the new heading
     within a reasonable time and tolerance.
   - Send a HDGALT_COMMAND with a new altitude 50m higher. Verify
     climb to new altitude.
   - Simulate a pilot stick input via `MANUAL_CONTROL`. Verify that
     mode switches to FBWA.
3. Register the test in the test framework so it's part of the
   default `arduplane` test run.

### Acceptance

- `Tools/autotest/autotest.py build.Plane test.Plane` runs the
  full plane test suite, and the new HDGALT test passes.
- The test catches regressions: temporarily break the heading PID
  and verify the test fails.

### Notes

- `Tools/autotest/` tests are slow (each runs a SITL flight). Keep
  the HDGALT test under ~2 minutes of sim time at default speedup.
- The test's tolerances should be lenient (e.g., ±10° heading
  hold, ±10m altitude hold) to avoid flakiness. Tight tolerances
  belong in flight tests, not regression tests.

---

## After step 9

The mode is functionally complete and tested. From here:

- Run the test suite a few times to catch any flakiness.
- Tune PID gains and parameter defaults for one or two known
  airframes (the default SITL plane is fine to start).
- Update `ArduPlane/mode_hdgalt.md` to reflect any spec changes
  that happened during implementation.
- Open the upstream PR. Reference the design doc, the test, and
  any Discord/Discourse discussion thread.

The reference flight director (separate repo) is a follow-on
project. Build it once HDGALT is merged or at least stable.

---

## General guidance for the coding assistant

- **Read before writing.** Each step references existing ArduPlane
  files as templates. Read them first. Match their style and
  patterns exactly. ArduPlane's reviewers will reject style
  divergence even if the code is correct.
- **Commit often.** Each step is one or a few commits. Each commit
  should be buildable and tested at least to the "it doesn't crash
  SITL on engagement" level.
- **Keep the design doc honest.** If you change something during
  implementation (a parameter name, a default value, a control flow
  detail), update `mode_hdgalt.md` in the same commit. The doc and
  the code must agree at every commit.
- **Maintain the `MODE_HDGALT_ENABLED` guard (since step 3).** The
  whole mode is feature-gated behind `MODE_HDGALT_ENABLED`
  (`mode.h`, default 1; `build_options.py` has the `Feature` row).
  Every step that adds HDGALT code MUST wrap it in
  `#if MODE_HDGALT_ENABLED`: new `mode_hdgalt.*` bodies, any new
  `Plane`/factory/`GOBJECT` lines, and especially any new
  `Mode::Number::HDGALT` `case` labels (e.g. the MAVLink command
  routing in step 6) — an unguarded `case` breaks the
  `--define MODE_HDGALT_ENABLED=0` build. Verify each step builds
  both with the macro at its default and with
  `./waf configure --board sitl --define MODE_HDGALT_ENABLED=0`.
- **AP_Param names are limited to 16 chars including the `HDGALT_`
  prefix.** Several design-doc names are abbreviated (see §4 table:
  `STALL_MGN`, `ASPD_MIN`, `CLIMB_RT`, `CLIMB_FT`, `PILOT_THR`,
  `DISENG_MODE`, …). Use the abbreviated name and keep the §4 table
  in sync as each owning step lands.
- **Don't over-engineer.** The design doc is deliberately minimal.
  If a piece of code doesn't have a corresponding design-doc
  justification, question whether it should exist. Resist the urge
  to add features.
- **When uncertain about ArduPlane conventions, ask.** Don't guess.
  The repo is large and has many internal conventions that aren't
  obvious from a single file. If something seems ambiguous, leave
  a `TODO(hdgalt)` comment and surface the question rather than
  silently picking one option.
