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
  sleep(2.0);                                       // abort before takeoff_countdown (5 s)

  mrs.state = mrs_msgs::msg::State::STATE_MANUAL; // OFFBOARD off, PX4 still "flying" until the stick is down
  if (!mrs.waitFor([&] { return !mrs.output_enabled; }, 2.0)) {
    return mrs.fail("automatic_start did not turn its output off on MANUAL");
  }
  sleep(2.0); // shorter than manual_resume_timeout

  mrs.state = mrs_msgs::msg::State::STATE_ARMED; // PX4 detected the landing
  if (!mrs.waitFor([&] { return mrs.output_enabled.load(); }, 4.0)) {
    return mrs.fail("automatic_start did not resume after a short MANUAL");
  }

  mrs.state = mrs_msgs::msg::State::STATE_OFFBOARD; // retry
  if (mrs.waitFor([&] { return mrs.takeoff_calls > 0; }, 3.0)) {
    return mrs.fail("automatic_start took off too early after the OFFBOARD retry -- countdown did not restart");
  }
  if (!mrs.waitFor([&] { return mrs.takeoff_calls > 0; }, 12.0)) {
    return mrs.fail("automatic_start did not take off after the OFFBOARD retry");
  }

  return mrs.disarm_calls == 0 ? true : mrs.fail("unexpected disarm");
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  Tester     tester;
  const bool result = tester.test();
  tester.sleep(1.0);
  tester.reportTestResult(result);
  tester.join();
}
