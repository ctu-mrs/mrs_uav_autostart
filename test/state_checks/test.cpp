#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <mrs_uav_autostart/state_checks.hpp>

using namespace mrs_uav_autostart;

struct Expected
{
  uint8_t state;
  bool    confirmed;
  bool    armed;
  bool    mrs_flying;
  bool    takeoff_interrupted;
};

// every State::STATE_*, in order (NO_LINK is the last one)
// clang-format off
const std::vector<Expected> all_states = {
  // state                      confirmed  armed  mrs_flying  takeoff_interrupted
  {State::STATE_UNKNOWN,        false,     false, false,      false},
  {State::STATE_DISARMED,       true,      false, false,      true},
  {State::STATE_ARMED,          true,      true,  false,      true},
  {State::STATE_OFFBOARD,       true,      true,  false,      false}, // may still read OFFBOARD right after the takeoff call
  {State::STATE_MANUAL,         true,      true,  false,      true},
  {State::STATE_TAKEOFF,        true,      true,  true,       false},
  {State::STATE_LAND,           true,      true,  true,       true},
  {State::STATE_RC_MODE,        true,      true,  true,       false},
  {State::STATE_HOVER,          true,      true,  true,       false},
  {State::STATE_GOTO,           true,      true,  true,       false},
  {State::STATE_TRAJECTORY,     true,      true,  true,       false},
  {State::STATE_MIDAIR,         true,      true,  true,       false},
  {State::STATE_EHOVER,         true,      true,  true,       true},
  {State::STATE_ELAND,          true,      true,  true,       true},
  {State::STATE_FAILSAFE,       true,      true,  true,       true},
  {State::STATE_NO_LINK,        false,     false, false,      false},
};
// clang-format on

TEST(StateChecks, coversAllStates) {
  ASSERT_EQ(all_states.size(), size_t(State::STATE_NO_LINK) + 1);
  for (size_t i = 0; i < all_states.size(); i++) {
    EXPECT_EQ(all_states[i].state, i);
  }
}

TEST(StateChecks, classifiesEveryState) {
  for (const auto &e : all_states) {
    SCOPED_TRACE("state " + std::to_string(e.state));
    EXPECT_EQ(isConfirmed(e.state), e.confirmed);
    EXPECT_EQ(isArmed(e.state), e.armed);
    EXPECT_EQ(isMrsFlying(e.state), e.mrs_flying);
    EXPECT_EQ(isTakeoffInterrupted(e.state), e.takeoff_interrupted);
  }
}

TEST(StateChecks, isPossiblyInTheAir) {

  // on the ground only when all three heuristics agree
  for (int mask = 0; mask < 8; mask++) {

    mrs_msgs::msg::PreflightStatus preflight;
    preflight.speed_ok  = mask & 1;
    preflight.height_ok = mask & 2;
    preflight.gyro_ok   = mask & 4;

    EXPECT_EQ(isPossiblyInTheAir(preflight), mask != 7)
        << "speed_ok=" << preflight.speed_ok << " height_ok=" << preflight.height_ok << " gyro_ok=" << preflight.gyro_ok;
  }
}
