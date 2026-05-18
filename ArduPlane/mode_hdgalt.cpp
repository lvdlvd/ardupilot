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

    climb_failed_latched = false;
    climb_fail_start_ms = 0;

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
    // TODO(hdgalt): release any latched failsafe state once it
    // exists (step 5). Nothing mode-specific to clean up yet.
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
    }

    // Never exceed the airframe roll limit either.
    const float bank_limit_cd = MIN(bank_limit_deg * 100.0f,
                                    (float)plane.roll_limit_cd);

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
}

#endif // MODE_HDGALT_ENABLED
