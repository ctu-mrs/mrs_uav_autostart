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
  mrs.position_valid = false; // SafetyAreaManager reports the UAV outside the safety area

  sleep(3.0); // DISARMED first, so the started-armed guard stays off

  mrs.state = mrs_msgs::msg::State::STATE_ARMED;

  if (mrs.waitFor([&] { return mrs.output_on_calls > 0 || mrs.disarm_calls > 0; }, 1.0)) {
    return mrs.fail("automatic_start turned output ON or disarmed before arm_to_output_timeout (1.5 s)");
  }

  if (!mrs.waitFor([&] { return mrs.disarm_calls > 0; }, 3.0)) {
    return mrs.fail("automatic_start did not disarm after arm_to_output_timeout");
  }

  return mrs.output_on_calls == 0 && mrs.output_off_calls == 0 && mrs.takeoff_calls == 0 ? true : mrs.fail("unexpected output ON / OFF or takeoff");
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  Tester     tester;
  const bool result = tester.test();
  tester.sleep(1.0);
  tester.reportTestResult(result);
  tester.join();
}
