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

  sleep(3.0); // long after the arming

  // the output drops OFF and every ON is refused: no disarm, the pilot must disarm and arm again
  mrs.refuse_output_on = true;
  mrs.output_enabled   = false;
  sleep(3.0);

  const int on_calls_in_rearm = mrs.output_on_calls;

  mrs.refuse_output_on = false;
  mrs.state            = mrs_msgs::msg::State::STATE_MANUAL; // a short manual flight must not resume the rearm requirement
  sleep(1.0);
  mrs.state = mrs_msgs::msg::State::STATE_ARMED;

  // a resume would ask for output ON within manual_resume_delay (1 s)
  if (mrs.waitFor([&] { return mrs.output_on_calls > on_calls_in_rearm; }, 4.0)) {
    return mrs.fail("automatic_start resumed after a short MANUAL although it required a disarm and arm");
  }

  return mrs.disarm_calls == 0 && mrs.takeoff_calls == 0 ? true : mrs.fail("unexpected disarm or takeoff");
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  Tester     tester;
  const bool result = tester.test();
  tester.sleep(1.0);
  tester.reportTestResult(result);
  tester.join();
}
