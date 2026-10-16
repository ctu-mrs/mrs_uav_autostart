#ifndef MRS_UAV_AUTOSTART_STATE_CHECKS_H
#define MRS_UAV_AUTOSTART_STATE_CHECKS_H

#include <cstdint>

#include <mrs_msgs/msg/state.hpp>

namespace mrs_uav_autostart
{

using State = mrs_msgs::msg::State;

// NO_LINK/UNKNOWN are ambiguous: neither armed nor disarmed is confirmed
inline bool isConfirmed(const uint8_t state) {
  return state != State::STATE_NO_LINK && state != State::STATE_UNKNOWN;
}

// DISARMED/NO_LINK/UNKNOWN mean "not confidently armed"
inline bool isArmed(const uint8_t state) {
  return isConfirmed(state) && state != State::STATE_DISARMED;
}

// MRS (a tracker) is already flying the UAV, so there is nothing left for us to start
// (MANUAL is not here: a pilot or the autopilot flies it without offboard, see AutomaticStart::handleManualPause())
inline bool isMrsFlying(const uint8_t state) {
  switch (state) {
  case State::STATE_TAKEOFF:
  case State::STATE_HOVER:
  case State::STATE_GOTO:
  case State::STATE_TRAJECTORY:
  case State::STATE_LAND:
  case State::STATE_RC_MODE:
  case State::STATE_MIDAIR:
  case State::STATE_EHOVER:
  case State::STATE_ELAND:
  case State::STATE_FAILSAFE:
    return true;
  default:
    return false;
  }
}

// the takeoff can no longer finish: MRS lost the UAV, or ControlManager switched to an emergency or landing;
// the state may still read OFFBOARD right after the takeoff call, NO_LINK/UNKNOWN are ambiguous
inline bool isTakeoffInterrupted(const uint8_t state) {
  switch (state) {
  case State::STATE_DISARMED:
  case State::STATE_ARMED:
  case State::STATE_MANUAL:
  case State::STATE_LAND:
  case State::STATE_EHOVER:
  case State::STATE_ELAND:
  case State::STATE_FAILSAFE:
    return true;
  default:
    return false;
  }
}

} // namespace mrs_uav_autostart

#endif // MRS_UAV_AUTOSTART_STATE_CHECKS_H
