#include "mode.h"
#include "Plane.h"

/*
  HDGALT: GA-style two-axis (heading + altitude) autopilot mode.

  This is the inert stub from step 2 of the implementation plan
  (ArduPlane/implementation_plan.md). The mode is selectable in SITL
  but performs no control yet; lateral, vertical, MAVLink command
  handling, state reporting and pilot override are added in later
  steps. See ArduPlane/mode_hdgalt.md for the design specification.
 */

bool ModeHdgAlt::_enter()
{
    // TODO(hdgalt): snapshot current heading/altitude as setpoints
    // (steps 3 and 4).
    return true;
}

void ModeHdgAlt::_exit()
{
    // TODO(hdgalt): release controllers and clear state when the
    // control logic exists (steps 3-7).
}

void ModeHdgAlt::update()
{
    // TODO(hdgalt): lateral PID, vertical TECS passthrough, command
    // handling, state emission and pilot override (steps 3-8).
    // Intentionally inert for now.
}
