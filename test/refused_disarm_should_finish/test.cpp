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
  mrs.topics_ok     = false; // a topic in the preflight topic check is missing
  mrs.refuse_disarm = true;  // and the HW API refuses the disarm

  // DISARMED first, so the started-armed guard stays off
  if (!mrs.waitForAutomaticStart()) {
    return mrs.fail("automatic_start never got DiagnosticsManager's data -- mock setup broken");
  }

  mrs.state = mrs_msgs::msg::State::STATE_ARMED;

  if (!mrs.waitFor([&] { return mrs.disarm_calls > 0; }, 4.0)) {
    return mrs.fail("automatic_start did not try to disarm after arm_to_output_timeout");
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
