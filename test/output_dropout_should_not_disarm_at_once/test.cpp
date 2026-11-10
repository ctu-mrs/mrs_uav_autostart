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

  sleep(3.0); // long after the arming: a timeout counted from the arming would fire right away

  // the output drops OFF and every ON is refused from now on, the disarm as well
  mrs.refuse         = true;
  mrs.output_enabled = false;

  if (mrs.waitFor([&] { return mrs.disarm_calls > 0; }, 1.0)) {
    return mrs.fail("automatic_start disarmed right after the output dropped OFF, not after arm_to_output_timeout");
  }

  if (!mrs.waitFor([&] { return mrs.disarm_calls > 0; }, 3.0)) { // arm_to_output_timeout is 1.5 s
    return mrs.fail("automatic_start did not try to disarm after arm_to_output_timeout");
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
