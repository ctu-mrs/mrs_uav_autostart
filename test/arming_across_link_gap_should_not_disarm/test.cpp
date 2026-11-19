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

  mrs.state = mrs_msgs::msg::State::STATE_NO_LINK; // the link drops while disarmed
  sleep(1.0);

  mrs.refuse_output_on = true;
  mrs.state            = mrs_msgs::msg::State::STATE_ARMED; // armed during the gap: automatic start did not see the arming

  // well past arm_to_output_timeout (1.5 s)
  if (mrs.waitFor([&] { return mrs.disarm_calls > 0; }, 4.0)) {
    return mrs.fail("automatic_start disarmed a UAV whose arming it did not see");
  }

  return mrs.takeoff_calls == 0 ? true : mrs.fail("unexpected takeoff");
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  Tester     tester;
  const bool result = tester.test();
  tester.sleep(1.0);
  tester.reportTestResult(result);
  tester.join();
}
