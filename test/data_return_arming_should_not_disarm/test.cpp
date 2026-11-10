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

  mrs.publish_general_robot_info = false; // DiagnosticsManager stops, stale after diagnostics_manager_timeout (3 s)
  sleep(3.5);

  // the first readings after the gap: armed (the pilot armed, maybe took off, while automatic start saw nothing)
  mrs.refuse_output_on           = true;
  mrs.state                      = mrs_msgs::msg::State::STATE_ARMED;
  mrs.publish_general_robot_info = true;

  // well past arm_to_output_timeout (1.5 s)
  if (mrs.waitFor([&] { return mrs.disarm_calls > 0; }, 4.0)) {
    return mrs.fail("automatic_start disarmed on an arming that came back together with DiagnosticsManager's data");
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
