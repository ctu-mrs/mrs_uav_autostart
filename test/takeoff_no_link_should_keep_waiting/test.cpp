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

  mrs.state = mrs_msgs::msg::State::STATE_OFFBOARD;
  if (!mrs.waitFor([&] { return mrs.takeoff_calls > 0; }, 10.0)) { // takeoff_countdown is 5 s
    return mrs.fail("automatic_start never triggered the takeoff");
  }

  mrs.state = mrs_msgs::msg::State::STATE_TAKEOFF;
  sleep(1.0);

  mrs.state = mrs_msgs::msg::State::STATE_NO_LINK; // ambiguous, not an interruption of the takeoff
  sleep(3.0);
  mrs.state = mrs_msgs::msg::State::STATE_UNKNOWN;
  sleep(2.0);

  mrs.state = mrs_msgs::msg::State::STATE_TAKEOFF; // the link is back
  sleep(1.0);

  mrs.state           = mrs_msgs::msg::State::STATE_HOVER;
  mrs.flying_normally = true; // the takeoff finished (checked in test.py from the log)
  sleep(2.0);

  if (mrs.output_off_calls > 0 || mrs.disarm_calls > 0 || mrs.takeoff_calls != 1) {
    return mrs.fail("automatic_start acted on the UAV during or after the NO_LINK / UNKNOWN gap");
  }

  return true;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  Tester     tester;
  const bool result = tester.test();
  tester.sleep(1.0);
  tester.reportTestResult(result);
  tester.join();
}
