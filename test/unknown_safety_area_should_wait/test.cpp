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
  mrs.position_known = false; // SafetyAreaManager has not reported yet
  mrs.position_valid = false; // only the default until SafetyAreaManager reports

  sleep(3.0); // DISARMED first, so the started-armed guard stays off

  mrs.state = mrs_msgs::msg::State::STATE_ARMED;

  // well past arm_to_output_timeout (1.5 s): acting on the default position_valid would disarm by now
  if (mrs.waitFor([&] { return mrs.output_on_calls > 0 || mrs.output_off_calls > 0 || mrs.disarm_calls > 0 || mrs.takeoff_calls > 0; }, 4.0)) {
    return mrs.fail("automatic_start acted on the UAV before the safety area check reported");
  }

  mrs.position_valid = true; // first, so no tick sees "known, but outside the safety area"
  mrs.position_known = true;

  if (!mrs.waitFor([&] { return mrs.output_enabled.load(); }, 3.0)) {
    return mrs.fail("automatic_start did not turn output ON once the safety area check reported");
  }

  if (mrs.waitFor([&] { return mrs.disarm_calls > 0; }, 2.0)) {
    return mrs.fail("automatic_start disarmed after the safety area check reported");
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
