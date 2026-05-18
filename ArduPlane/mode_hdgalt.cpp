#include "mode.h"
#include "Plane.h"

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

    // placeholder vertical: hold the altitude we engaged at via
    // TECS. Step 4 replaces this with an explicit altitude target.
    plane.set_target_altitude_current();

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

    // --- vertical: placeholder (step 4 replaces with explicit
    // HDGALT altitude target fed to TECS) ---
    plane.update_fbwb_speed_height();
}

#endif // MODE_HDGALT_ENABLED
