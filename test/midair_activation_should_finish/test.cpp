#include <rclcpp/rclcpp.hpp>

#include <mrs_msgs/msg/state.hpp>

#include <mrs_uav_testing/test_generic.h>

#include <mrs_mock.hpp>

class Tester : public mrs_uav_testing::TestGeneric {

public:
  bool test(void);
};

bool Tester::test(void) {

  MrsMock mrs(node_);

  // DISARMED first, so the started-armed guard stays off
  if (!mrs.waitForAutomaticStart()) {
    return mrs.fail("automatic_start never got DiagnosticsManager's data -- mock setup broken");
  }

  mrs.state = mrs_msgs::msg::State::STATE_ARMED;
  if (!mrs.waitFor([&] { return mrs.output_enabled.load(); }, 10.0)) {
    return mrs.fail("automatic_start never enabled output after arming -- mock setup broken");
  }

  mrs.state = mrs_msgs::msg::State::STATE_MANUAL; // pilot takes off without offboard
  if (!mrs.waitFor([&] { return !mrs.output_enabled; }, 2.0)) {
    return mrs.fail("automatic_start did not turn its output off on MANUAL");
  }

  sleep(1.0); // a short MANUAL, the pause is still resumable

  // UavManager's mid-air activation: control output ON (by UavManager, not through automatic start), then
  // MidairActivationTracker holds the UAV while the autopilot is switched to OFFBOARD
  const int on_calls_before_activation  = mrs.output_on_calls;
  const int off_calls_before_activation = mrs.output_off_calls;

  mrs.output_enabled = true;
  sleep(0.5); // still MANUAL with output ON: automatic start must only warn, not turn it OFF

  // stays MIDAIR to the end: test.py's "already flying" check must come from this state, not a later HOVER
  mrs.state = mrs_msgs::msg::State::STATE_MIDAIR;
  sleep(3.0);

  if (!mrs.output_enabled || mrs.output_off_calls != off_calls_before_activation) {
    return mrs.fail("automatic_start turned output OFF during the mid-air activation");
  }

  if (mrs.output_on_calls != on_calls_before_activation) {
    return mrs.fail("automatic_start toggled output during the mid-air activation");
  }

  return mrs.disarm_calls == 0 && mrs.takeoff_calls == 0 ? true : mrs.fail("unexpected disarm / takeoff");
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  Tester     tester;
  const bool result = tester.test();
  tester.sleep(1.0);
  tester.reportTestResult(result);
  tester.join();
}
