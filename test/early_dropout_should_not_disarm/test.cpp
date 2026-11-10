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
  if (!mrs.waitFor([&] { return mrs.output_enabled.load(); }, 5.0)) {
    return mrs.fail("automatic_start never enabled output after arming -- mock setup broken");
  }

  // the output drops OFF soon after the arming and every ON is refused: the timeout restarts at the dropout, not at the
  // arming, and fires about 2 s after the arming
  sleep(0.5);
  mrs.refuse_output_on = true;
  mrs.output_enabled   = false;

  if (mrs.waitFor([&] { return mrs.disarm_calls > 0; }, 4.0)) {
    return mrs.fail("automatic_start disarmed on a timeout that started at a dropout, not at the arming");
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
