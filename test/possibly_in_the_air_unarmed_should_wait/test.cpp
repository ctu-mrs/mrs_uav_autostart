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
  mrs.moving = true;

  sleep(5.0); // DISARMED and possibly in the air (e.g. carried around): only warns, keeps waiting
  if (mrs.output_on_calls > 0 || mrs.output_off_calls > 0 || mrs.disarm_calls > 0 || mrs.takeoff_calls > 0) {
    return mrs.fail("automatic_start acted on the UAV while not armed and possibly in the air");
  }

  mrs.moving = false; // put down
  sleep(1.0);

  mrs.state = mrs_msgs::msg::State::STATE_ARMED; // still waiting, not finished: a normal start
  if (!mrs.waitFor([&] { return mrs.output_enabled.load(); }, 10.0)) {
    return mrs.fail("automatic_start did not enable output after arming -- it stopped waiting while not armed and possibly in the air");
  }

  mrs.moving = true; // armed and possibly in the air: finishes and turns its output OFF
  if (!mrs.waitFor([&] { return !mrs.output_enabled; }, 2.0)) {
    return mrs.fail("automatic_start did not turn its output off when armed and possibly in the air");
  }

  mrs.moving = false;
  mrs.state  = mrs_msgs::msg::State::STATE_OFFBOARD; // finished: must not count down and take off
  sleep(7.0);                                        // longer than takeoff_countdown (5 s)

  return mrs.disarm_calls == 0 && mrs.takeoff_calls == 0 && !mrs.output_enabled ? true : mrs.fail("unexpected disarm / takeoff / output ON after finishing");
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  Tester     tester;
  const bool result = tester.test();
  tester.sleep(1.0);
  tester.reportTestResult(result);
  tester.join();
}
