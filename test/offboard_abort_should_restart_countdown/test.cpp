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

  sleep(3.0); // DISARMED first, so the started-armed guard stays off

  mrs.state = mrs_msgs::msg::State::STATE_ARMED;
  if (!mrs.waitFor([&] { return mrs.output_enabled.load(); }, 10.0)) {
    return mrs.fail("automatic_start never enabled output after arming -- mock setup broken");
  }

  mrs.state = mrs_msgs::msg::State::STATE_OFFBOARD; // countdown starts
  sleep(3.0);                                       // abort before takeoff_countdown (5 s)

  mrs.state = mrs_msgs::msg::State::STATE_ARMED; // OFFBOARD off without MANUAL, back to waiting for OFFBOARD
  sleep(2.0);                                    // OFFBOARD + ARMED longer than takeoff_countdown
  if (mrs.takeoff_calls > 0 || !mrs.output_enabled || mrs.output_off_calls > 0) {
    return mrs.fail("automatic_start took off / turned output OFF after the pilot aborted the countdown");
  }

  mrs.state = mrs_msgs::msg::State::STATE_OFFBOARD; // retry
  if (mrs.waitFor([&] { return mrs.takeoff_calls > 0; }, 4.0)) {
    return mrs.fail("automatic_start took off too early after the OFFBOARD retry -- countdown did not restart");
  }
  if (!mrs.waitFor([&] { return mrs.takeoff_calls > 0; }, 6.0)) {
    return mrs.fail("automatic_start did not take off after the OFFBOARD retry");
  }

  return mrs.disarm_calls == 0 && mrs.output_off_calls == 0 ? true : mrs.fail("unexpected disarm / output OFF");
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  Tester     tester;
  const bool result = tester.test();
  tester.sleep(1.0);
  tester.reportTestResult(result);
  tester.join();
}
