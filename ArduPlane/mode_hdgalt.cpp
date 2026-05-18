#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>

#if MODE_HDGALT_ENABLED

/*
  HDGALT: GA-style two-axis (heading + altitude) autopilot mode.

  Step 3 implements the lateral controller: a heading -> bank outer
  loop with a static bank limit and a dynamic stall-margin limit.
  On engagement the current heading is snapshotted and held.
  Vertical control is a placeholder (hold engagement altitude via
  TECS, as CRUISE does); step 4 replaces it with an explicit
  altitude target. MAVLink command handling and pilot override come
  in later steps. See ArduPlane/mode_hdgalt.md for the design.
 */

const AP_Param::GroupInfo ModeHdgAlt::var_info[] = {

    // @Param: BANK_MAX
    // @DisplayName: HDGALT maximum bank angle
    // @Description: Hard upper limit on commanded bank angle in HDGALT mode.
    // @Range: 5 45
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("BANK_MAX", 1, ModeHdgAlt, bank_max_deg, 25),

    // @Param: STALL_MGN
    // @DisplayName: HDGALT stall margin
    // @Description: Minimum ratio of current airspeed to stall airspeed maintained when clipping the commanded bank in turns.
    // @Range: 1.1 2.0
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("STALL_MGN", 2, ModeHdgAlt, stall_margin, 1.3),

    // @Param: ASPD_MIN
    // @DisplayName: HDGALT minimum airspeed
    // @Description: Protective airspeed floor used by the stall-margin bank limit. Airframe-specific (typically about 1.3x stall speed); the default is a placeholder and should be tuned.
    // @Range: 5 30
    // @Increment: 0.5
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("ASPD_MIN", 3, ModeHdgAlt, airspeed_min, 10.0),

    // @Param: CLIMB_RT
    // @DisplayName: HDGALT default climb rate
    // @Description: Default climb/descent rate cap used when capturing a new altitude. The flight director may override this per command.
    // @Range: 0.5 10
    // @Increment: 0.1
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("CLIMB_RT", 5, ModeHdgAlt, climb_rate_default, 2.0),

    // @Param: PITCH_MAX
    // @DisplayName: HDGALT maximum pitch
    // @Description: Hard upper pitch limit passed to TECS in HDGALT mode.
    // @Range: 5 25
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("PITCH_MAX", 6, ModeHdgAlt, pitch_max_deg, 10),

    // @Param: CLIMB_FT
    // @DisplayName: HDGALT climb-fail timeout
    // @Description: Time TECS must remain unable to follow the demanded climb or descent (with altitude error outside the deadband) before HDGALT latches the climb-unachievable failsafe and holds the current altitude.
    // @Range: 2 30
    // @Increment: 1
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("CLIMB_FT", 7, ModeHdgAlt, climb_fail_timeout, 10.0),

    // @Param: TURN_RATE
    // @DisplayName: HDGALT default turn rate
    // @Description: Default turn-rate cap used when capturing a new heading. The flight director may override this per command. The commanded bank is additionally limited so the turn rate does not exceed this.
    // @Range: 1 15
    // @Increment: 0.5
    // @Units: deg/s
    // @User: Standard
    AP_GROUPINFO("TURN_RATE", 8, ModeHdgAlt, turn_rate_default, 3.0),

    // @Group: HD_
    // @Path: ../libraries/AC_PID/AC_PID.cpp
    AP_SUBGROUPINFO(heading_pid, "HD_", 4, ModeHdgAlt, AC_PID),

    AP_GROUPEND
};

ModeHdgAlt::ModeHdgAlt() :
    Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeHdgAlt::_enter()
{
    // snapshot current heading as the setpoint (design doc 3.2)
    heading_cmd_deg = wrap_360(degrees(plane.ahrs.get_yaw_rad()));

    heading_pid.reset_I();
    heading_pid.reset_filter();
    last_update_ms = AP_HAL::millis();

    // snapshot current barometric (MSL) altitude as the setpoint
    // (design doc 3.2). current_loc.alt is AMSL in cm.
    altitude_cmd_m = plane.current_loc.alt * 0.01f;
    altitude_target_m = altitude_cmd_m;
    climb_rate_cmd_mps = climb_rate_default;
    turn_rate_cmd_dps = turn_rate_default;

    pending.valid = false;

    climb_failed_latched = false;
    climb_fail_start_ms = 0;
    turn_derated = false;
    last_state_ms = 0;

    // initialise the shared target-altitude struct (slope offset,
    // terrain flags) consistently. HDGALT is barometric MSL only
    // (design doc 1.3 / 3.1): never terrain-follow.
    plane.set_target_altitude_current();
#if AP_TERRAIN_AVAILABLE
    plane.target_altitude.terrain_following = false;
#endif

    return true;
}

void ModeHdgAlt::_exit()
{
    // Nothing mode-specific to release: the lateral PID and the
    // shared TECS/target-altitude state are reset by the next mode's
    // entry and by Mode::reset_controllers().
}

// Apply a (possibly partial) command immediately. Per design doc
// 5.1, an axis is updated only if its flag bit is set; unset axes
// keep their current value. Any applied command clears the
// climb-unachievable latch (design doc 3.4 / 6.4).
void ModeHdgAlt::apply_command(uint16_t flags, float heading_deg,
                               float turn_rate_dps, float altitude_m,
                               float climb_rate_mps)
{
    if (flags & HDGALT_COMMAND_FLAG_HEADING) {
        heading_cmd_deg = wrap_360(heading_deg);
    }
    if (flags & HDGALT_COMMAND_FLAG_TURN_RATE) {
        turn_rate_cmd_dps = fabsf(turn_rate_dps);
    }
    if (flags & HDGALT_COMMAND_FLAG_ALTITUDE) {
        altitude_cmd_m = altitude_m;
    }
    if (flags & HDGALT_COMMAND_FLAG_CLIMB_RATE) {
        climb_rate_cmd_mps = fabsf(climb_rate_mps);
    }

    // A new command is the FD acknowledging state: clear the
    // climb-unachievable latch and re-evaluate against the new target.
    climb_failed_latched = false;
    climb_fail_start_ms = 0;
}

void ModeHdgAlt::handle_hdgalt_command(uint32_t start_time_boot_ms,
                                       uint16_t flags, float heading_deg,
                                       float turn_rate_dps, float altitude_m,
                                       float climb_rate_mps)
{
    const uint32_t now = AP_HAL::millis();

    if (start_time_boot_ms == 0 || start_time_boot_ms <= now) {
        if (start_time_boot_ms != 0 && start_time_boot_ms < now) {
            gcs().send_text(MAV_SEVERITY_INFO,
                            "HDGALT: command %u ms late, applied now",
                            (unsigned)(now - start_time_boot_ms));
        }
        apply_command(flags, heading_deg, turn_rate_dps, altitude_m,
                      climb_rate_mps);
        // an immediate command supersedes any queued one
        pending.valid = false;
        return;
    }

    // Future command: single-slot queue, replaces any pending one
    // (design doc 2.3 / 5.1).
    pending.valid = true;
    pending.start_ms = start_time_boot_ms;
    pending.flags = flags;
    pending.heading_deg = heading_deg;
    pending.turn_rate_dps = turn_rate_dps;
    pending.altitude_m = altitude_m;
    pending.climb_rate_mps = climb_rate_mps;
}

// Heading -> bank outer loop, clipped by the static bank limit and
// the dynamic stall-margin limit (design doc 3.3 / 8.3). Returns a
// bank command in centidegrees.
float ModeHdgAlt::compute_bank_command_cd(float heading_error_rad, float dt)
{
    // PID: positive error (need to turn towards higher heading)
    // produces positive (right) bank, mirroring g2.guidedHeading.
    const float bank_request_cd = heading_pid.update_error(heading_error_rad, dt);

    // Static bank limit.
    float bank_limit_deg = bank_max_deg;

    // Dynamic stall-margin limit: require
    //   V_current >= V_s_turn * stall_margin,  V_s_turn = V_s/sqrt(cos phi)
    // => cos(phi) >= (airspeed_min * stall_margin / V_current)^2
    const float airspeed = plane.smoothed_airspeed;
    if (is_positive(airspeed)) {
        const float ratio = (airspeed_min * stall_margin) / airspeed;
        float bank_stall_max_deg;
        if (ratio >= 1.0f) {
            bank_stall_max_deg = 0.0f;   // can't safely bank at all
        } else {
            bank_stall_max_deg = degrees(acosf(sq(ratio)));
        }
        bank_limit_deg = MIN(bank_limit_deg, bank_stall_max_deg);

        // Turn-rate cap (design doc 3.3): a coordinated turn has
        // omega = g*tan(phi)/V, so the bank that yields exactly the
        // commanded turn rate is phi = atan(omega*V/g).
        if (is_positive(turn_rate_cmd_dps)) {
            const float bank_turn_max_deg =
                degrees(atanf(radians(turn_rate_cmd_dps) * airspeed /
                              GRAVITY_MSS));
            bank_limit_deg = MIN(bank_limit_deg, bank_turn_max_deg);
        }
    }

    // Never exceed the airframe roll limit either.
    const float bank_limit_cd = MIN(bank_limit_deg * 100.0f,
                                    (float)plane.roll_limit_cd);

    // Derated if the PID wanted more bank than the active limit
    // allows: the achieved turn rate is then below the commanded one.
    turn_derated = fabsf(bank_request_cd) > bank_limit_cd;

    return constrain_float(bank_request_cd, -bank_limit_cd, bank_limit_cd);
}

void ModeHdgAlt::update()
{
    const uint32_t now = AP_HAL::millis();
    float dt = (now - last_update_ms) * 0.001f;
    last_update_ms = now;
    if (!is_positive(dt) || dt > 1.0f) {
        // first tick after engage, or a long stall: use a nominal dt
        dt = 0.02f;
    }

    // --- apply a queued future command whose time has arrived ---
    if (pending.valid && now >= pending.start_ms) {
        apply_command(pending.flags, pending.heading_deg,
                      pending.turn_rate_dps, pending.altitude_m,
                      pending.climb_rate_mps);
        pending.valid = false;
    }

    // --- lateral: heading hold ---
    const float error_rad = wrap_PI(radians(heading_cmd_deg) -
                                    plane.ahrs.get_yaw_rad());
    plane.nav_roll_cd = compute_bank_command_cd(error_rad, dt);
    plane.update_load_factor();

    // --- climb/descent-unachievable detection (design doc 3.4 / 6.4) ---
    // TECS freezes its height demand when it can't follow the
    // commanded climb/descent (pitch or throttle saturated). If that
    // persists with the altitude error (to the *commanded* altitude)
    // outside the deadband for HDGALT_CLIMB_FT seconds, latch the
    // failsafe: clip the target to the current altitude and warn once.
    const float DEADBAND_M = 5.0f;
    const float current_alt_m = plane.current_loc.alt * 0.01f;
    const float alt_err_m = altitude_cmd_m - current_alt_m;
    if (!climb_failed_latched) {
        if (fabsf(alt_err_m) > DEADBAND_M &&
            plane.TECS_controller.height_demand_limited()) {
            if (climb_fail_start_ms == 0) {
                climb_fail_start_ms = now;
            } else if ((now - climb_fail_start_ms) >
                       (uint32_t)(climb_fail_timeout * 1000.0f)) {
                climb_failed_latched = true;
                // hold the altitude we can actually reach
                altitude_cmd_m = current_alt_m;
                altitude_target_m = current_alt_m;
                gcs().send_text(MAV_SEVERITY_WARNING,
                                "HDGALT: climb unachievable, holding altitude");
            }
        } else {
            climb_fail_start_ms = 0;
        }
    }
    // The latch clears only when a new HDGALT_COMMAND arrives
    // (wired in step 6).

    // --- vertical: hold commanded MSL altitude via TECS ---
    // Slew the target toward the commanded altitude at no more than
    // the climb-rate cap; TECS tracks the moving setpoint, so the
    // cap is enforced without a per-call TECS climb limit (design
    // doc 3.4). climb_rate_cmd_mps is a magnitude.
    const float max_step_m = fabsf(climb_rate_cmd_mps) * dt;
    altitude_target_m += constrain_float(altitude_cmd_m - altitude_target_m,
                                         -max_step_m, max_step_m);

    plane.target_altitude.amsl_cm = lroundf(altitude_target_m * 100.0f);
    plane.reset_offset_altitude();

    // hard pitch cap on the TECS side (design doc 3.4 / HDGALT_PITCH_MAX)
    plane.TECS_controller.set_pitch_max(pitch_max_deg);

    // apply TECS pitch + throttle demands, as LOITER does in its
    // non-stick-mixing path
    plane.calc_nav_pitch();
    plane.calc_throttle();

    // --- periodic HDGALT_STATE (design doc 5.2): 1 Hz ---
    if (now - last_state_ms >= 1000) {
        last_state_ms = now;
        send_hdgalt_state();
    }
}

// Fill and broadcast HDGALT_STATE to all active GCS channels. The
// _target fields are setpoints, _current are measured, _actual are
// what the AP is achieving (so the FD can see clipping/derating).
void ModeHdgAlt::send_hdgalt_state()
{
    const float heading_now = wrap_360(degrees(plane.ahrs.get_yaw_rad()));
    const float alt_now_m = plane.current_loc.alt * 0.01f;

    // capture vs hold per axis (same deadbands the controllers use)
    const float hdg_err = fabsf(wrap_180(heading_cmd_deg - heading_now));
    const float alt_err = fabsf(altitude_cmd_m - alt_now_m);
    const bool cap_lat = hdg_err > 5.0f;
    const bool cap_vert = alt_err > 5.0f;
    uint8_t capture_state = HDGALT_CAPTURE_HOLD;
    if (cap_lat && cap_vert) {
        capture_state = HDGALT_CAPTURE_BOTH;
    } else if (cap_lat) {
        capture_state = HDGALT_CAPTURE_LATERAL;
    } else if (cap_vert) {
        capture_state = HDGALT_CAPTURE_VERTICAL;
    }

    uint16_t warnings = 0;
    if (climb_failed_latched) {
        warnings |= HDGALT_WARNING_CLIMB_UNACHIEVABLE;
    }
    if (turn_derated) {
        warnings |= HDGALT_WARNING_TURN_DERATED;
    }
    // approaching the protective airspeed floor (disengage in 6.5)
    if (plane.smoothed_airspeed < airspeed_min * 1.15f) {
        warnings |= HDGALT_WARNING_AIRSPEED_LOW;
    }

    mavlink_hdgalt_state_t packet{};
    packet.time_boot_ms = AP_HAL::millis();
    packet.flags = HDGALT_STATE_FLAG_HEADING_ACTIVE |
                   HDGALT_STATE_FLAG_ALTITUDE_ACTIVE;
    packet.queued_start_time_ms = pending.valid ? pending.start_ms : 0;
    packet.heading_target = heading_cmd_deg;
    packet.heading_current = heading_now;
    packet.turn_rate_commanded = turn_rate_cmd_dps;
    packet.turn_rate_actual = degrees(plane.ahrs.get_yaw_rate_earth());
    packet.altitude_target = altitude_cmd_m;
    packet.altitude_current = alt_now_m;
    packet.climb_rate_commanded = climb_rate_cmd_mps;
    packet.climb_rate_actual = plane.barometer.get_climb_rate();
    packet.airspeed_current = plane.smoothed_airspeed;
    packet.capture_state = capture_state;
    packet.warnings = warnings;

    gcs().send_to_active_channels(MAVLINK_MSG_ID_HDGALT_STATE,
                                  (const char *)&packet);
}

#endif // MODE_HDGALT_ENABLED
