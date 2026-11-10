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
  // flying_normally stays false: the takeoff never finishes

  sleep(3.0); // DISARMED first, so the started-armed guard stays off

  mrs.state = mrs_msgs::msg::State::STATE_ARMED;
  if (!mrs.waitFor([&] { return mrs.output_enabled.load(); }, 10.0)) {
    return mrs.fail("automatic_start never enabled output after arming -- mock setup broken");
  }

  mrs.state = mrs_msgs::msg::State::STATE_OFFBOARD;
  if (!mrs.waitFor([&] { return mrs.takeoff_calls > 0; }, 10.0)) { // takeoff_countdown is 5 s
    return mrs.fail("automatic_start never triggered the takeoff");
  }

  mrs.state = mrs_msgs::msg::State::STATE_TAKEOFF; // stays in TAKEOFF, longer than takeoff_timeout (3 s)
  sleep(5.0);

  // finishing is checked in test.py from the log; here, it must not have touched the UAV on the way out
  if (mrs.output_off_calls > 0 || mrs.disarm_calls > 0 || mrs.takeoff_calls != 1) {
    return mrs.fail("automatic_start acted on the UAV after the takeoff timed out");
  }

  return true;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  Tester     tester;
  const bool result = tester.test();
  tester.sleep(1.0);
  tester.reportTestResult(result);
  tester.join();
}
