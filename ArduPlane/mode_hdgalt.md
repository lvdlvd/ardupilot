# HDGALT Mode Design Proposal

**Status:** Design draft. Implementation in progress on branch `hdgaltmode`.

# HDGALT — A GA-Style Two-Axis Autopilot Mode for ArduPlane

A new ArduPlane flight mode providing the general-aviation
autopilot building block: capture and hold a commanded magnetic
heading and barometric altitude, with bank and climb-rate limits,
under the direction of an external flight director communicating
over MAVLink. The mode does not know about waypoints, position,
or navigation; spatial reasoning happens in the flight director.

This is the design specification. Sections 1–3 explain what and
why; sections 4–7 specify the interface and behavior; section 8
maps to ArduPlane implementation. Section 9 describes the
reference flight director; sections 10–11 cover testing and
out-of-scope items.

---

## 1. Motivation and goals

### 1.1 The general-aviation factoring

In light general aviation the avionics community has converged on
a clean division of labor:

- The **autopilot** holds simple state setpoints: keep wings level
  at this bank, hold this pitch, hold this heading, hold this
  altitude. It reacts continuously to the airplane's current state.
  It does not know about geography or geometry.
- The **flight director** computes those setpoints from intent —
  "fly this leg," "intercept this radial," "begin descent in 30
  seconds." It knows about positions, plans, timing. It updates
  setpoints discretely.

This separation has good properties:

- Each layer is independently testable. The AP can be flown
  manually by a human-as-FD; the FD can be developed against a
  known-good AP.
- The AP's certification surface is small and well-defined: scalar
  setpoint regulators with documented envelopes.
- Different FDs can drive the same AP: a human pilot setting bugs
  on a panel, a VOR/DME-based FD, an INS, a custom visual
  navigation system.
- The AP carries no navigation-database dependency. Replacing the
  FD does not require recertifying the AP.

ArduPilot's existing modes mostly conflate these two layers. AUTO
knows about waypoints (FD-level) and executes them via roll, pitch,
and throttle (AP-level). GUIDED accepts position targets via MAVLink
(FD-level inputs) but again does all the geometry internally. There
is no mode in ArduPlane today that is *pure AP* in the GA sense: a
setpoint holder with no spatial awareness.

HDGALT fills that gap.

### 1.2 Goals

- **A pure setpoint regulator.** Commanded magnetic heading and
  commanded barometric altitude. No waypoints, no position, no
  navigation. The mode can engage and operate without GPS.
- **Composes with existing controllers.** HDGALT does not replace
  TECS or the attitude controllers. It provides the outer loop on
  the lateral axis and passes the vertical setpoint into TECS.
- **External flight director.** Setpoints come from an FD running
  outside ArduPilot, communicating via MAVLink. The FD can be a
  human-driven GCS, a simple time-based scheduler, a VOR/NDB/DME radio
  navigation or a visual-nav system, or anything else.
- **Pilot is supreme.** Any stick input by the pilot disengages
  HDGALT immediately. No partial-override modes, no cooldowns, no
  pilot-vs-FD races. The pilot re-engages via the standard mode
  switch when ready.
- **Envelope-protected.** Commanded turn rates are clipped to
  maintain a configurable margin above stall. Commanded climb rates
  are honored if achievable; if not, the AP holds altitude and
  raises a flag.
- **Suitable for community contribution.** Clean, minimal, no
  exotic dependencies. Intended to add value to ArduPlane's mode portfolio
  for users who want a GA-style autopilot building block.

### 1.3 Non-goals

- **Throttle control as a HDGALT concern.** The vertical control is
  delegated to TECS, which coordinates throttle and pitch as it
  does in AUTO. The FD may command airspeed separately via the
  existing `MAV_CMD_DO_CHANGE_SPEED`; that command is outside the
  HDGALT message family.
- **Position-aware behavior.** HDGALT never reads GPS or any
  position estimate. No "fly toward this point" capability. The FD
  does all such reasoning.
- **Mission interpretation.** ArduPlane's mission storage and
  upload remain available — a GCS can upload a mission, it persists
  to EEPROM, the FD can read it via standard MAVLink — but the
  HDGALT mode itself does not walk mission items. The FD reads the
  mission and translates it into setpoint commands.
- **Multi-vehicle, VTOL, copter applicability.** HDGALT is a fixed-
  wing ArduPlane mode. The energy and bank-stall models do not
  apply to multirotors or VTOLs in hover.
- **Wind compensation.** Heading is held, not ground track. The FD
  is responsible for adjusting commanded heading to compensate for
  wind drift, if it cares about ground track.

## 2. Conceptual model

### 2.1 Layered architecture

```
+---------------------------+
|        Pilot              |  ultimate authority; any input → disengage
+---------------------------+
              |
+---------------------------+
|   Flight director (FD)    |  external; speaks MAVLink
|   - reads mission, GPS    |
|   - computes setpoints    |
|   - schedules commands    |
+---------------------------+
              | HDGALT_COMMAND        | DO_CHANGE_SPEED
              v                       v
+---------------------------+    +---------+
|      HDGALT mode          |    |  TECS   |  airspeed regulator,
|   - heading PID → bank    |    |         |  coordinates throttle + pitch
|   - altitude → TECS       |--->|         |  for vertical
+---------------------------+    +---------+
              | bank command          | pitch + throttle
              v                       v
+---------------------------+    +---------+
|   Roll attitude loop      |    | Plane   |
|   (existing ArduPlane)    |    | servos  |
+---------------------------+    +---------+
```

The FD has two channels into the autopilot: the new HDGALT command
family for heading and altitude, and the existing speed-change
mechanism for airspeed. These are orthogonal. The FD may use one
or both. If the FD never sends a speed command, TECS uses
`AIRSPEED_CRUISE` as its default airspeed target.

### 2.2 What HDGALT controls

HDGALT-the-mode directly controls **bank angle** (and thus roll) via
its lateral PID. It controls **altitude** indirectly by passing the
target altitude to TECS, which coordinates pitch and throttle to
achieve it.

The mode does not:
- Read GPS or any position state
- Modify TECS internals
- Override the pilot's throttle quadrant when disengaged

The mode does:
- Read attitude (yaw, current bank) for heading control
- Read airspeed (for stall-margin protection)
- Read altitude (passed-through indirectly via TECS targets)

### 2.3 Time and scheduling

The flight director may send commands timestamped to execute in the
future. The autopilot maintains a single-slot pending-command
queue: at most one future command is buffered. When the FD sends a
new command and a pending one exists, the new one replaces the
pending one (allowing the FD to update its plan).

Clock alignment is via the standard `SYSTEM_TIME` MAVLink message,
which the autopilot already emits at 1 Hz. The FD records the
autopilot's `time_boot_ms` from incoming SYSTEM_TIME messages,
estimates the AP's clock from its own, and uses absolute AP-time
values in HDGALT_COMMAND `start_time_boot_ms` fields. A value of 0
means "execute immediately."

### 2.4 Pilot is supreme

Any pilot stick input that exceeds a deadband causes HDGALT to
disengage immediately and switch to a stabilized fallback mode
(default FBWA). This is a single rule with no exceptions: there is
no partial-override mode, no nudge-and-hold-the-new-state, no
cooldown after pilot release. The disengage is total.

The pilot re-engages HDGALT through the standard mode switch when
they choose. On re-engagement, the mode snapshots current heading
and altitude as the initial setpoints and waits for the FD to send
new commands. This is the same behavior as initial engagement.

## 3. The HDGALT mode

### 3.1 Setpoint vocabulary

HDGALT operates with two primary scalar setpoints:

- **Heading** ψ_cmd: magnetic heading, 0–360°. The plane turns to
  this heading and holds it.
- **Altitude** h_cmd: barometric altitude (mean sea level), metres.
  The plane climbs or descends to this altitude and holds it.

Each setpoint has an associated rate cap that controls capture
aggressiveness:

- **Turn rate** ω_cmd: degrees per second. The AP turns at this
  rate (or less, if envelope-clipped) during heading capture.
  Default is `HDGALT_TURN_RATE_DEFAULT`, typically 3.0°/s (rate 1).
- **Climb rate** ḣ_cmd: metres per second. The AP climbs or
  descends at this rate during altitude capture. Default is
  `HDGALT_CLIMB_RATE_DEFAULT`, typically 2.0 m/s.

Each rate is either supplied by the FD per command, or inherited
from the parameter default. The FD can omit either field; the AP
uses the default for unspecified fields.

### 3.2 Engagement

When HDGALT is engaged (via mode switch, RC channel, or MAVLink
command), the AP snapshots the current state:

- ψ_cmd ← current magnetic heading
- h_cmd ← current barometric altitude
- ω_cmd ← HDGALT_TURN_RATE_DEFAULT
- ḣ_cmd ← HDGALT_CLIMB_RATE_DEFAULT

The mode then holds this state. If the FD sends a new HDGALT_COMMAND,
the relevant setpoints update; otherwise the snapshot persists. This
allows the mode to be useful without any FD at all: a human pilot
can engage HDGALT at any time and the airplane will hold the heading
and altitude it had at engagement.

There is no negotiation between AP and FD: the FD watches the AP's
mode via the standard HEARTBEAT message, and begins sending
HDGALT_COMMANDs when it sees HDGALT mode is active. When the AP
disengages (pilot input, mode change), the FD stops sending.

### 3.3 Lateral behavior

The lateral controller is a PID outer loop on heading error,
producing a commanded bank angle, which feeds the existing roll
attitude controller.

The commanded bank angle is clipped by two limits:

1. The static **bank limit** `HDGALT_BANK_MAX` (degrees). This is
   a structural/comfort cap on bank angle, typically 25–30° for a
   normal-category airplane.

2. The dynamic **stall-margin** limit. Stall speed in a banked turn
   increases with load factor: V_s_turn = V_s × √(1/cos φ). The AP
   computes the bank angle at which V_s_turn would equal
   V_current / HDGALT_STALL_MARGIN, and clips at that.

The actual bank command is `min(commanded, BANK_MAX, stall_margin_max)`.
From the resulting bank angle and current airspeed, the AP computes
the actual achievable turn rate (ω = g·tan(φ)/V) and reports it in
HDGALT_STATE so the FD can observe the difference between requested
and achieved.

In hold mode (heading error within deadband), the controller
operates linearly with small corrections. In capture mode (large
heading error), the bank command saturates at the active limit and
the airplane turns at the maximum allowable rate until error
diminishes.

The lateral controller has no separate "capture" and "hold" code
paths: a single PID with appropriate saturation handles both
regimes naturally.

### 3.4 Vertical behavior

HDGALT passes the altitude setpoint and climb-rate cap to TECS as
its altitude target and pitch-rate limits. TECS handles the
coordination of pitch and throttle to achieve the target.

The AP does not implement its own altitude PID. The vertical
controller is, for all practical purposes, TECS — already tuned and
tested in ArduPlane's existing modes.

The climb-rate cap `HDGALT_CLIMB_RT` (or per-command override)
bounds the capture rate. It is enforced by slewing the altitude
setpoint fed to TECS at that rate rather than by a TECS internal
limit (see 8.2 for why); TECS tracks the moving setpoint and so
does not exceed the cap during capture.

**Climb failure detection.** TECS already limits its own height
demand when it cannot follow a commanded climb or descent (pitch or
throttle saturated against its limit while the demanded altitude is
moving away). A small, library-wide accessor
`AP_TECS::height_demand_limited()` exposes this existing internal
state (a ~4-line, side-effect-free addition). HDGALT monitors it:
if it is true *and* the altitude error (to the **commanded**
altitude) stays outside a 5 m deadband for longer than
`HDGALT_CLIMB_FT` seconds, HDGALT latches the **climb-unachievable**
failsafe:

- `altitude_cmd_m` and the slewed target are clipped to the current
  altitude. TECS therefore stops trying to climb and stabilises
  level flight.
- A warning bit `CLIMB_UNACHIEVABLE` will be set in HDGALT_STATE
  (step 7; step 5 carries the latched bool).
- A STATUSTEXT is emitted once at `MAV_SEVERITY_WARNING`:
  "HDGALT: climb unachievable, holding altitude".

Note on the signal: because the climb-rate cap is enforced by
*slewing* the fed setpoint (§8.2), the TECS demand is rising every
tick while capturing a higher target, so this detector fires during
the capture of any unreachable target — not only when the raw
command step exceeds climb capability. When the held target is
constant and reachable, the demand is not increasing and the
detector correctly stays clear.

The latch clears when a new HDGALT_COMMAND arrives, at which point
the AP re-evaluates against the new target. This gives the FD a
clean signal: "I commanded altitude X; AP could not reach it;
acknowledged via new command." Symmetrically for descent if the
airframe is too slick to descend at the requested rate.

### 3.5 Pilot input handling

Any pilot stick input beyond a deadband disengages HDGALT and
switches the autopilot to `HDGALT_DISENGAGE_MODE` (default FBWA).

The deadband is `HDGALT_PILOT_THRESHOLD`, a fraction of stick
travel, default 0.10.

Triggers, applied each control loop tick:

- **Roll stick** deflection from neutral exceeds threshold.
- **Pitch stick** deflection from neutral exceeds threshold.
- **Throttle stick** position deviates from TECS-commanded throttle
  by more than the threshold. (Rationale: the pilot's hand may
  rest on the throttle quadrant without moving it, but a deliberate
  push or pull causes the position to diverge from what TECS is
  asking for, which triggers the disengage.)

Yaw (rudder) stick is **not** a disengage trigger. Rudder passes
through as in FBWA (yaw damper active, rudder input augments).
Pilots use rudder for coordination and crosswind correction during
normal AP-engaged flight without intending to disengage.

Re-engagement is the standard mode switch path: there is no special
"resume" state.

## 4. Parameters

All parameters live under the `HDGALT_` prefix.

All parameters use the `HDGALT_` prefix and are registered the
ArduPlane way: a `GOBJECT(mode_hdgalt, "HDGALT_", ModeHdgAlt)` in
`Parameters.cpp` plus a `ModeHdgAlt::var_info[]` table (the same
pattern `ModeTakeoff`/`TKOFF_` uses).

ArduPlane parameter names are limited to 16 characters including the
prefix, so several design names from earlier drafts are abbreviated
in the implementation. The table below gives the **implemented**
name; the abbreviated ones are marked.

| Parameter (implemented)     | Units | Default             | Purpose                                          |
| --------------------------- | ----- | ------------------- | ------------------------------------------------ |
| `HDGALT_BANK_MAX`           | deg   | 25                  | Hard upper bank angle limit                      |
| `HDGALT_PITCH_MAX`          | deg   | 10                  | Hard upper pitch angle limit (TECS-side)         |
| `HDGALT_TURN_RATE` *(was TURN_RATE_DEFAULT)*  | deg/s | 3.0   | Default turn rate; FD may override per command   |
| `HDGALT_CLIMB_RT` *(was CLIMB_RATE_DEFAULT)*  | m/s   | 2.0   | Default climb rate; FD may override per command  |
| `HDGALT_STALL_MGN` *(was STALL_MARGIN)*       | ratio | 1.3   | Min airspeed / stall airspeed in turns           |
| `HDGALT_ASPD_MIN` *(was AIRSPEED_MIN)*        | m/s   | (airframe-specific) | Protective airspeed floor / disengage threshold |
| `HDGALT_CLIMB_FT` *(was CLIMB_FAIL_TIMEOUT)*  | s     | 10    | Time before latching climb-unachievable failsafe |
| `HDGALT_PILOT_THR` *(was PILOT_THRESHOLD)*    | ratio | 0.10  | Stick deadband; above this, disengage            |
| `HDGALT_DISENG_MODE` *(was DISENGAGE_MODE)*   | mode  | FBWA  | Where to go on disengage                         |
| `HDGALT_HD_*`               | —     | see below           | Heading→bank `AC_PID` gains (P/I/D/FF/IMAX/FLT…) |

The `HDGALT_HD_` group is the heading-hold controller's `AC_PID`,
exposed as an `AP_SUBGROUPINFO` (so `HDGALT_HD_P`, `HDGALT_HD_I`,
…). Its starting gains are copied from ArduPlane's existing,
proven GUIDED heading-hold controller (`g2.guidedHeading`):
`{P=5000, I=0, D=0, FF=0, IMAX=10, FLTT=5, FLTE=5, FLTD=5}`.

Names marked *(was …)* are abbreviations of earlier-draft names
forced by the 16-character limit; the abbreviated names land with
their owning implementation step (TURN_RATE/CLIMB_RT/CLIMB_FT in
steps 4–6, PILOT_THR/DISENG_MODE in step 8). Only `HDGALT_BANK_MAX`,
`HDGALT_STALL_MGN`, `HDGALT_ASPD_MIN` and `HDGALT_HD_*` exist as of
step 3.

Default values are starting points. Each is tunable.

`HDGALT_ASPD_MIN` is airframe-specific (typically ~1.3 × V_s).
It is the airspeed below which HDGALT will not operate: if airspeed
drops below this, the AP disengages with a STATUSTEXT warning.

## 5. MAVLink interface

Two new messages, defined in the ArduPilot dialect
`modules/mavlink/message_definitions/v1.0/ardupilotmega.xml`
(proposed for upstream common.xml after stabilization).

As implemented:

- `HDGALT_COMMAND` is message ID **11061**.
- `HDGALT_STATE` is message ID **11062**.
- `PLANE_MODE_HDGALT = 27` is added to the `PLANE_MODE` enum (the
  next free value after `PLANE_MODE_AUTOLAND = 26`); this matches
  `Mode::Number::HDGALT` on the ArduPlane side.
- The `flags`, `warnings`, and `capture_state` fields are backed by
  the `HDGALT_COMMAND_FLAGS`, `HDGALT_STATE_FLAGS`,
  `HDGALT_WARNING`, and `HDGALT_CAPTURE_STATE` enums.

Note: mavgen reorders struct/wire fields by descending type size;
the field lists below are the logical contract, not the wire
layout.

### 5.1 HDGALT_COMMAND (FD → AP)

```
message HDGALT_COMMAND
  uint32  start_time_boot_ms    # AP time at which to begin; 0 = immediate
  uint16  flags                 # bit 0: heading_set
                                # bit 1: turn_rate_set
                                # bit 2: altitude_set
                                # bit 3: climb_rate_set
  float   heading_deg           # magnetic, 0-360
  float   turn_rate_dps         # magnitude only; AP picks direction (shortest)
  float   altitude_m            # barometric MSL
  float   climb_rate_mps        # magnitude; sign derived from altitude delta
```

For each axis, the FD may either set the flag and provide a value,
or leave the flag clear and let the AP keep its current value
(initial snapshot or previous command).

The `start_time_boot_ms` field carries the AP's own time-since-boot
in milliseconds, as seen by the FD via SYSTEM_TIME. A value of 0
means "execute on receipt." A nonzero future value queues the
command; a nonzero past value executes immediately and is logged as
late.

Only one future command may be queued at any time. Sending a new
HDGALT_COMMAND with `start_time_boot_ms` in the future replaces any
previously-queued command. Sending one with `start_time_boot_ms = 0`
executes immediately and clears any queued command.

### 5.2 HDGALT_STATE (AP → FD, periodic)

Emitted at 1 Hz minimum while HDGALT is engaged.

```
message HDGALT_STATE
  uint32  time_boot_ms          # AP timestamp
  uint16  flags                 # which axes are actively controlled
                                # bit 0: heading active
                                # bit 1: altitude active
  uint32  queued_start_time_ms  # 0 if no command queued
  float   heading_target        # current commanded heading (deg)
  float   heading_current       # measured heading (deg)
  float   turn_rate_commanded   # what FD asked for / default (deg/s)
  float   turn_rate_actual      # what AP is actually doing (deg/s)
  float   altitude_target       # current commanded altitude (m)
  float   altitude_current      # measured altitude (m)
  float   climb_rate_commanded  # commanded climb rate cap (m/s)
  float   climb_rate_actual     # actual vertical speed (m/s)
  float   airspeed_current      # for FD situational awareness (m/s)
  uint8   capture_state         # 0=hold, 1=cap-lateral, 2=cap-vertical, 3=both
  uint16  warnings              # bit 0: CLIMB_UNACHIEVABLE
                                # bit 1: TURN_DERATED  (stall-margin clip active)
                                # bit 2: AIRSPEED_LOW  (approaching disengage)
                                # (room for more)
```

The `_actual` fields let the FD detect when its commands are being
clipped or derated. Example: FD commands a 6°/s turn, but the AP
clips to 4°/s to maintain stall margin. The FD sees
`turn_rate_actual = 4.0` with `warnings & TURN_DERATED != 0` and
can respond (e.g., command throttle to gain airspeed first, then
re-issue the turn).

### 5.3 Reused existing messages

- `SYSTEM_TIME` (MAVLink ID 2): AP→FD clock alignment, 1 Hz.
- `HEARTBEAT` (ID 0): mode reporting; FD watches `custom_mode` to
  know when HDGALT is engaged.
- `GLOBAL_POSITION_INT` (ID 33): AP→FD position reporting. The AP
  computes this from its own EKF; in HDGALT mode this is for the
  FD's use, not consumed by HDGALT itself.
- `VFR_HUD` (ID 74): AP→FD basic flight data.
- `MISSION_*` family: standard mission upload/download to the AP;
  read by the FD, not interpreted by the AP in HDGALT mode.
- `COMMAND_LONG` with `MAV_CMD_DO_CHANGE_SPEED`: FD→AP airspeed
  command, handled by ArduPlane's existing speed control, fed to
  TECS as airspeed target.

## 6. Failure modes

### 6.1 FD goes silent

The AP keeps its current setpoints indefinitely. The plane continues
flying the last commanded heading and altitude. No timeout-driven
mode change.

The HDGALT_STATE message continues to emit, allowing any monitoring
GCS to detect the FD silence (no new commands appearing, no FD
log lines, etc.).

This is the GA convention: an autopilot holds its bug settings until
told otherwise. A datalink failure does not endanger the airplane;
it just means the FD's plan stops updating. The pilot retains the
override authority described in section 3.5.

### 6.2 No command ever received

The engagement-time snapshot persists. The plane flies the heading
and altitude it had at engagement. Default turn and climb rates
from parameters.

### 6.3 Command exceeds envelope

The AP clips the command at the active envelope limit and executes
the clipped value. `HDGALT_STATE` reports both the commanded and
actual values, plus a warning bit. The FD can detect and respond.

Specific cases:
- Heading change requested with turn rate exceeding `HDGALT_BANK_MAX`
  at current airspeed: bank clipped, turn rate derated, `TURN_DERATED`
  warning set.
- Altitude change requested with climb rate exceeding `HDGALT_PITCH_MAX`
  in TECS: TECS limits climb rate, AP reflects actual.
- Heading commanded value outside 0–360: rejected, command ignored,
  warning logged. (FDs should not do this; bug if they do.)

### 6.4 Climb or descent unachievable

After `HDGALT_CLIMB_FT` seconds of TECS height-demand limiting
(`height_demand_limited()`) with altitude error outside the 5 m
deadband, the AP latches the failsafe:

- `altitude_cmd_m` and the slewed target are clipped to the current
  altitude.
- `CLIMB_UNACHIEVABLE` warning bit set in HDGALT_STATE (step 7;
  step 5 holds the latched bool internally).
- One STATUSTEXT emitted at MAV_SEVERITY_WARNING.

Latch clears when a new HDGALT_COMMAND arrives (wired in step 6;
step 5 leaves a documented hook).

### 6.5 Airspeed below minimum

If indicated airspeed drops below `HDGALT_AIRSPEED_MIN` for more
than a short debounce (1 second), the AP disengages immediately.
A STATUSTEXT is emitted: "HDGALT: airspeed below minimum, disengaging."

The disengage target is `HDGALT_DISENGAGE_MODE`. The pilot or FD
must reassess the situation before re-engaging.

### 6.6 Pilot stick override

See section 3.5. Disengage is immediate, no debounce, no STATUSTEXT
(the mode change is visible in HEARTBEAT).

### 6.7 Unexpected mode transition

If something else in ArduPlane causes a mode change while HDGALT is
engaged (e.g., a failsafe triggered by GCS link loss configured to
switch to RTL), HDGALT's mode-exit handler clears state and
releases the controllers. The other mode takes over. The FD sees
the mode change in HEARTBEAT and stops sending HDGALT commands.

## 7. Mission integration

ArduPlane's existing mission storage is reused without modification.
A GCS uploads a mission via the standard MISSION_COUNT /
MISSION_REQUEST / MISSION_ITEM_INT protocol; the mission persists
to EEPROM; it survives reboots.

**HDGALT mode does not walk mission items.** The mission, when
HDGALT is the active mode, is an inert data structure stored in
the AP. It does not influence the AP's behavior.

The flight director reads the mission via the standard MAVLink
protocol (the same one a GCS would use). It interprets mission
items according to its own logic, and translates them into a
sequence of HDGALT_COMMANDs and (optionally) DO_CHANGE_SPEED
commands.

This means:

- A user can plan a mission in Mission Planner or QGroundControl,
  upload it normally, and have the FD fly it in HDGALT mode. The
  GCS does not need to know about HDGALT; it just uploads a
  mission as usual.
- The same mission can be flown by different FDs (a dead-reckoning
  FD, a VFR visual-nav FD, an IFR procedure FD) without changing
  the mission file.
- The mission survives FD restart: if the FD process dies and
  restarts, it re-reads the mission from the AP's storage.

The AP-side responsibility is minimal: keep accepting and storing
missions through the standard protocol, expose them through the
standard protocol, but don't otherwise interpret them while HDGALT
is the active mode.

## 8. ArduPlane implementation

### 8.1 New mode class

A new file `ArduPlane/mode_hdgalt.cpp` with corresponding header
declaration. The class follows the `Mode` base interface used by
all ArduPlane modes.

The whole mode (enum value, class, `.cpp` body, `Plane` member and
`friend`, factory case, `GOBJECT`, and the `mode_number()` switch
cases) is wrapped in `#if MODE_HDGALT_ENABLED`, defined in
`mode.h` defaulting to `1`, with a `Feature` row in
`Tools/scripts/build_options.py` — the same feature-gate convention
`MODE_AUTOLAND_ENABLED` uses, so the mode can be compiled out on
flash-constrained boards.

```cpp
#if MODE_HDGALT_ENABLED
class ModeHdgAlt : public Mode {
public:
    ModeHdgAlt();
    Number mode_number() const override { return Number::HDGALT; }
    const char *name() const override { return "HDGALT"; }
    const char *name4() const override { return "HDGA"; }

    void update() override;
    bool does_auto_throttle() const override { return true; }
    void update_target_altitude() override {};   // we manage it

    void handle_HDGALT_COMMAND(const mavlink_message_t &msg);

    static const struct AP_Param::GroupInfo var_info[];
    AP_Float bank_max_deg;   // HDGALT_BANK_MAX
    AP_Float stall_margin;   // HDGALT_STALL_MGN
    AP_Float airspeed_min;   // HDGALT_ASPD_MIN
    // heading->bank, mirrors g2.guidedHeading (AC_PID)
    AC_PID heading_pid{5000.0, 0.0, 0.0, 0.0, 10.0, 5.0, 5.0, 5.0, 0.0};

protected:
    bool _enter() override;
    void _exit() override;

    // HDGALT is an in-air mode; arming directly in it is refused.
    bool _pre_arm_checks(size_t buflen, char *buffer) const override { return false; }

private:
    // setpoints
    float heading_cmd_deg;
    float altitude_cmd_m;       // step 4
    float turn_rate_cmd_dps;    // step 6
    float climb_rate_cmd_mps;   // step 4/6
    uint32_t last_update_ms;

    // pending future command (step 6)
    uint32_t pending_start_time_ms;
    bool pending_command_valid;
    // ... pending values

    // failsafe state (step 5)
    bool climb_failed_latched;
    uint32_t climb_fail_start_ms;

    float compute_bank_command_cd(float heading_error_rad, float dt);
};
#endif // MODE_HDGALT_ENABLED
```

`Plane` must also declare `friend class ModeHdgAlt;` (ArduPlane
gates access to protected members such as `ahrs` per-mode via
explicit friend declarations).

ArduPlane note (differs from Copter): ArduPlane's `Mode` base has
**no** `requires_GPS()` or `allows_arming()` virtuals. The design
intent maps to ArduPlane idioms as follows:

- *"not an arming-from mode"*: override `_pre_arm_checks()` to
  return `false`. ArduPlane's prearm path
  (`AP_Arming_Plane` → `Mode::pre_arm_checks`) then refuses arming
  in this mode with the message `"<MODE> mode not armable"`. Several
  existing modes (e.g. TAKEOFF, AUTOLAND) use this same idiom.
- *"does not require position"*: nothing to declare. ArduPlane modes
  simply do not consult GPS unless they need it; HDGALT never reads
  position (see sections 1.3 and 2.2), so `requires_GPS()` has no
  ArduPlane analogue and is not needed.
- `is_landing()` is not overridden: the `Mode` base default is
  already `false`, and only landing modes override it.

The mode is registered the usual ArduPlane way: a `Mode::Number`
enum value (`HDGALT = 27`), a `ModeHdgAlt` member on `Plane`, a
case in `Plane::mode_from_mode_num()`, and `case`s added to the
`mode_number()` switch statements in `GCS_Plane.cpp`,
`GCS_MAVLink_Plane.cpp` and `events.cpp` (ArduPlane builds these
`-Wswitch -Werror`). No build-manifest edit is needed: ArduPlane's
`wscript` globs its sources. In `GCS_Plane.cpp` HDGALT advertises
yaw and Z-altitude sensor control but **not** XY-position control,
consistent with its no-position-awareness design.

### 8.2 Reused controllers

The lateral path:
- HDGALT's heading `AC_PID` produces a bank command in centidegrees
  (see 8.3); `update()` writes it to `plane.nav_roll_cd` and calls
  `plane.update_load_factor()`. The base `Mode::run()` then runs the
  existing `stabilize_roll/pitch/yaw` attitude controllers — already
  tuned, composes with the yaw damper, handles servo saturation.

The vertical path (as implemented — ArduPlane has no
`tecs->set_altitude_target_m`-style API; targets are driven through
the shared `Plane::target_altitude` struct):

- On `_enter()`, the current AMSL altitude (`current_loc.alt`) is
  snapshotted into `altitude_cmd_m`; `set_target_altitude_current()`
  initialises the shared struct and terrain-following is forced off
  (HDGALT is barometric MSL only, design 1.3 / 3.1).
- Each tick, a separate `altitude_target_m` is slewed toward
  `altitude_cmd_m` by at most `climb_rate_cmd_mps * dt`. **This is
  how the climb-rate cap is enforced**: TECS tracks the moving
  setpoint, so no per-call TECS climb-rate limiter is needed (none
  exists; `TECS_CLMB_MAX` is a global param). The slewed value is
  written to `plane.target_altitude.amsl_cm`.
- `TECS_controller.set_pitch_max(HDGALT_PITCH_MAX)` applies the hard
  pitch cap (same per-tick setter `takeoff` uses).
- `plane.calc_nav_pitch()` and `plane.calc_throttle()` apply the
  TECS pitch/throttle demands, mirroring LOITER's non-stick-mixing
  path. `does_auto_throttle()` returns true so the scheduled TECS
  update runs; `update_target_altitude()` is overridden empty so the
  generic nav profile does not fight the held target.
- Climb-unachievable detection (step 5) reads
  `plane.TECS_controller.height_demand_limited()` — a new, minimal,
  library-wide accessor exposing TECS's existing height-demand
  freeze. See §3.4 and §6.4.

The yaw path is unchanged from FBWA: rudder input passes through,
yaw damper active.

### 8.3 The lateral PID outer loop

The heading→bank controller is **not** a hand-rolled PID and not
the legacy `libraries/PID` class (unused in modern ArduPlane).
ArduPlane already ships a proven heading→bank controller — the
GUIDED mode's `g2.guidedHeading` (`AC_PID`, error in **radians**,
output in **centidegrees** of bank). HDGALT mirrors it exactly:

```cpp
// member, defaults copied from g2.guidedHeading
AC_PID heading_pid{5000.0, 0.0, 0.0, 0.0, 10.0, 5.0, 5.0, 5.0, 0.0};

// per tick, in centidegrees:
float ModeHdgAlt::compute_bank_command_cd(float err_rad, float dt) {
    const float bank_request_cd = heading_pid.update_error(err_rad, dt);

    float bank_limit_deg = bank_max_deg;            // HDGALT_BANK_MAX

    // Dynamic stall-margin limit (design §3.3):
    // require V >= V_s_turn * margin, V_s_turn = V_s / sqrt(cos phi)
    // => cos(phi) >= (airspeed_min * stall_margin / V)^2
    const float airspeed = plane.smoothed_airspeed;
    if (is_positive(airspeed)) {
        const float ratio = (airspeed_min * stall_margin) / airspeed;
        const float bank_stall_max_deg =
            (ratio >= 1.0f) ? 0.0f : degrees(acosf(sq(ratio)));
        bank_limit_deg = MIN(bank_limit_deg, bank_stall_max_deg);
    }
    // never exceed the airframe roll limit either
    const float bank_limit_cd =
        MIN(bank_limit_deg * 100.0f, (float)plane.roll_limit_cd);

    return constrain_float(bank_request_cd, -bank_limit_cd, bank_limit_cd);
}
```

The heading error is `wrap_PI(radians(heading_cmd_deg) -
ahrs.get_yaw_rad())`. Note `ahrs.get_yaw_rad()` is earth-frame
(true) yaw, not magnetic; for heading *hold* this is correct
because the engagement snapshot and the running error use the same
reference, so declination cancels. The true-vs-magnetic distinction
only matters for the FD-facing values in `HDGALT_STATE` and is
addressed there (step 7). Airspeed comes from `plane.smoothed_airspeed`
(the same source the existing stall-protection code uses).

Total new code is on the order of 100–200 lines. Most of the mode
is parameter-handling, message-parsing, and state machine
plumbing.

### 8.4 MAVLink registration

The new messages live in a custom dialect XML file initially. For
upstream contribution, they'd be proposed for common.xml. The
custom dialect lets us iterate the message format during development
without requiring upstream coordination.

ArduPlane registers a handler for HDGALT_COMMAND in the GCS_MAVLINK
infrastructure, routing it to the active mode if HDGALT is engaged.
If HDGALT is not engaged, the command is logged and dropped (the
FD shouldn't be sending commands when HDGALT isn't active, but the
behavior is defined).

HDGALT_STATE is emitted by the mode's `update()` at the configured
stream rate.

### 8.5 Parameters

Standard ArduPlane parameter registration in `Parameters.cpp`. Each
parameter gets documentation strings, range hints, units, default
values, as is conventional for ArduPlane params.

## 9. Reference flight director

A standalone Go program in a separate repository (style modeled on
ardusitl). Connects to the AP via MAVLink. Reads mission, computes
leg sequences, sends HDGALT_COMMANDs.

### 9.1 Capabilities

The reference FD:

- Connects to the AP via TCP MAVLink.
- Receives SYSTEM_TIME at 1 Hz; tracks AP clock offset.
- Receives GLOBAL_POSITION_INT continuously; tracks AP position.
- Receives HEARTBEAT; only operates while AP mode is HDGALT.
- On mode transition into HDGALT: reads the current mission from
  AP via MISSION_REQUEST_LIST.
- For each mission leg (waypoint i → waypoint i+1):
  - Computes great-circle bearing as commanded heading.
  - Computes great-circle distance / cruise airspeed as time on
    leg.
  - Schedules a HDGALT_COMMAND at `current_time + time_on_leg` to
    set the next leg's heading and altitude.
- Sends DO_CHANGE_SPEED at leg start if airspeed should change.

This FD does no wind compensation. The plane will drift off course
when wind isn't aligned with the leg. That's acceptable as a
demonstration; more capable FDs (VFR visual-nav, radio-nav) are
planned but separate.

### 9.2 What the reference FD is for

- **Integration test:** demonstrates that HDGALT works end-to-end
  with an external FD over MAVLink.
- **Reference implementation:** for anyone writing their own FD,
  shows the message exchange pattern.
- **Baseline for comparison:** future FDs can be benchmarked
  against the reference (e.g., "VFR FD reduces cross-track error
  by X% vs no-wind dead reckoning").

## 10. Test plan

### 10.1 Unit tests

- Heading PID: error wrap, saturation, integral windup.
- Stall-margin calculator: edge cases (very low airspeed, very
  high airspeed, zero airspeed).
- Bank limit math: known-input known-output cases.
- Pending-command queue: replace behavior, time ordering.

These can be tested in pure C++ without an autopilot binary.

### 10.2 SITL tests

The following SITL tests are foreseen:
- A reference-FD process launched alongside ardupilot, communicating
  via MAVLink. 
- Test missions exercising: a simple straight leg; a single turn;
  a multi-leg pattern; a climb; a descent; the climb-failsafe
  scenario (commanded altitude beyond achievable).

Pass criteria:
- Heading and altitude are captured and held within documented
  tolerances.
- Bank derating triggers correctly at low airspeed.
- Climb-failsafe latches and clears as specified.
- Pilot stick simulation (via RC override) disengages immediately.

### 10.3 Real-flight checklist

Before in-aircraft use, the standard ArduPlane procedure: bench
tests, hangar power-on tests, low-speed taxi, eventual flight test
with safety pilot. The mode-specific tests:

- Disengage trigger latency: deliberately apply stick input, verify
  disengage occurs within one control loop tick (~50 ms typically).
- Bank limit: command a heading change that requires bank near
  HDGALT_BANK_MAX; verify the bank limit is honored.
- Climb failure: command altitude beyond what the airplane can
  reach at current throttle; verify the latched failsafe.
- Pilot supremacy: verify that pilot stick input always wins, in
  all axes, under all conditions.

The full EAS-style flight test plan would be developed as a
separate document. The intent is that HDGALT passes the same level
of testing as any other ArduPlane mode.

## 11. What's not in scope

- **Throttle control as a HDGALT concern.** TECS handles throttle,
  airspeed targets come from `DO_CHANGE_SPEED`, not HDGALT_COMMAND.
- **Position-aware behavior.** HDGALT never reads position. Spatial
  reasoning is the FD's job.
- **Mission interpretation in the AP.** Missions are stored but not
  walked by HDGALT.
- **Wind compensation.** Heading hold means magnetic heading, not
  ground track. The FD adjusts heading commands if it cares about
  ground track.
- **Three-axis operation.** A future complementary `SPEED_HOLD`
  mode could provide airspeed regulation via throttle, composing
  with HDGALT for full three-axis. Not part of this design.
- **VTOL, multirotor.** Fixed-wing only. The energy and bank-stall
  models do not apply elsewhere.
- **Pilot-AP shared control.** Any pilot input → disengage. No
  control-wheel-steering, no nudge-and-hold, no override
  cooldowns.
- **AP-side path planning.** Even simple things like "fly an
  S-turn" or "perform a holding pattern" are the FD's
  responsibility. The AP just executes setpoints.
