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

  // toggle_output answers the ON only after 3 s, refused: the pilot could take off meanwhile
  mrs.refuse_output_on = true;
  mrs.output_on_delay  = 3.0;
  mrs.state            = mrs_msgs::msg::State::STATE_ARMED;

  // the call returns 3 s after the arming, past arm_to_output_timeout (1.5 s) + 1 s
  if (mrs.waitFor([&] { return mrs.disarm_calls > 0; }, 6.0)) {
    return mrs.fail("automatic_start disarmed long after the arming, once a slow toggle_output call returned");
  }

  return mrs.output_on_calls > 0 && mrs.takeoff_calls == 0 ? true : mrs.fail("toggle_output was never called, or an unexpected takeoff");
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  Tester     tester;
  const bool result = tester.test();
  tester.sleep(1.0);
  tester.reportTestResult(result);
  tester.join();
}
