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

  mrs.moving = true; // carried around, possibly in the air
  sleep(1.0);

  mrs.state = mrs_msgs::msg::State::STATE_ARMED; // armed while still possibly in the air: finishes before turning output ON
  sleep(2.0);

  mrs.moving = false;
  mrs.state  = mrs_msgs::msg::State::STATE_OFFBOARD; // finished: must not count down and take off
  sleep(7.0);                                        // longer than takeoff_countdown (5 s)

  return mrs.output_on_calls == 0 && mrs.output_off_calls == 0 && mrs.disarm_calls == 0 && mrs.takeoff_calls == 0
             ? true
             : mrs.fail("automatic_start acted on the UAV after it was armed while possibly in the air");
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  Tester     tester;
  const bool result = tester.test();
  tester.sleep(1.0);
  tester.reportTestResult(result);
  tester.join();
}
