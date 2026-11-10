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

  sleep(3.0); // long after the arming

  // the output drops OFF and every ON is refused from now on; the UAV may be flying manually, reading ARMED
  mrs.refuse_output_on = true;
  mrs.output_enabled   = false;

  // well past arm_to_output_timeout (1.5 s)
  if (mrs.waitFor([&] { return mrs.disarm_calls > 0; }, 4.0)) {
    return mrs.fail("automatic_start disarmed after the output dropped OFF long after the arming");
  }

  // the output comes back ON (ControlManager): automatic start turned it ON before, so it turns it OFF again
  const int off_calls_before = mrs.output_off_calls;
  mrs.output_enabled         = true;
  if (!mrs.waitFor([&] { return mrs.output_off_calls > off_calls_before && !mrs.output_enabled; }, 2.0)) {
    return mrs.fail("automatic_start left its output ON while the pilot must disarm and arm again");
  }

  // the pilot disarms and arms again: a fresh start
  mrs.state = mrs_msgs::msg::State::STATE_DISARMED;
  sleep(1.0);

  mrs.refuse_output_on = false;
  mrs.state            = mrs_msgs::msg::State::STATE_ARMED;
  if (!mrs.waitFor([&] { return mrs.output_enabled.load(); }, 5.0)) {
    return mrs.fail("automatic_start did not turn output ON after the pilot disarmed and armed again");
  }

  return mrs.disarm_calls == 0 && mrs.takeoff_calls == 0 ? true : mrs.fail("unexpected disarm or takeoff");
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  Tester     tester;
  const bool result = tester.test();
  tester.sleep(1.0);
  tester.reportTestResult(result);
  tester.join();
}
