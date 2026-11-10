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
  mrs.topics_ok = false; // output is never turned ON, the arm-to-output timeout runs

  // DISARMED first, so the started-armed guard stays off
  if (!mrs.waitForAutomaticStart()) {
    return mrs.fail("automatic_start never got DiagnosticsManager's data -- mock setup broken");
  }

  mrs.state = mrs_msgs::msg::State::STATE_ARMED; // WAITING_FOR_OUTPUT, arm_to_output_timeout (4 s) starts
  sleep(1.0);

  mrs.publish_general_robot_info = false; // stale after diagnostics_manager_timeout (1 s)

  if (mrs.waitFor([&] { return mrs.disarm_calls > 0; }, 4.0)) {
    return mrs.fail("automatic_start disarmed during the general_robot_info gap");
  }

  mrs.publish_general_robot_info = true; // DiagnosticsManager is back, 5 s since the arming: past arm_to_output_timeout

  if (mrs.waitFor([&] { return mrs.disarm_calls > 0; }, 2.5)) {
    return mrs.fail("automatic_start disarmed right after general_robot_info was back, the arm-to-output timeout did not restart");
  }

  if (!mrs.waitFor([&] { return mrs.disarm_calls > 0; }, 4.0)) {
    return mrs.fail("automatic_start did not disarm after arm_to_output_timeout since general_robot_info was back");
  }

  return mrs.output_on_calls == 0 && mrs.takeoff_calls == 0 ? true : mrs.fail("unexpected output ON or takeoff");
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  Tester     tester;
  const bool result = tester.test();
  tester.sleep(1.0);
  tester.reportTestResult(result);
  tester.join();
}
