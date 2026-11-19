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
  if (!mrs.waitFor([&] { return mrs.output_enabled.load(); }, 10.0)) {
    return mrs.fail("automatic_start never enabled output after arming -- mock setup broken");
  }

  mrs.state = mrs_msgs::msg::State::STATE_MANUAL; // a short manual flight
  if (!mrs.waitFor([&] { return !mrs.output_enabled; }, 2.0)) {
    return mrs.fail("automatic_start did not turn its output off on MANUAL");
  }
  sleep(1.0); // shorter than manual_resume_timeout

  // back on the ground, or still flying with the link lost in between and airborne unknown: both read ARMED
  mrs.refuse_output_on = true;
  mrs.state            = mrs_msgs::msg::State::STATE_ARMED;

  // resumes after manual_resume_delay (1 s), then well past arm_to_output_timeout (1.5 s)
  if (mrs.waitFor([&] { return mrs.disarm_calls > 0; }, 5.0)) {
    return mrs.fail("automatic_start disarmed after resuming from MANUAL");
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
