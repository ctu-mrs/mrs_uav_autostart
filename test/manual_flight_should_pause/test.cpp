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

  mrs.state = mrs_msgs::msg::State::STATE_MANUAL; // pilot takes off without offboard
  if (!mrs.waitFor([&] { return !mrs.output_enabled; }, 2.0)) {
    return mrs.fail("automatic_start did not turn its output off on MANUAL");
  }

  // counted from the output OFF: a duplicate ON sent before MANUAL was seen (control_info lagging) lands before it
  const int on_calls_before_pause = mrs.output_on_calls; // output must stay OFF throughout the pause, not only when sampled

  sleep(7.0); // MANUAL longer than manual_resume_timeout (5 s) -> a real flight

  mrs.state = mrs_msgs::msg::State::STATE_NO_LINK; // link loss is not a disarm: must keep the pause and the rearm requirement
  sleep(2.0);
  if (mrs.output_enabled || mrs.disarm_calls > 0 || mrs.takeoff_calls > 0) {
    return mrs.fail("automatic_start resumed / disarmed / took off during NO_LINK after a real flight");
  }

  mrs.state = mrs_msgs::msg::State::STATE_ARMED; // landed, still armed
  sleep(3.0);
  if (mrs.output_enabled || mrs.disarm_calls > 0 || mrs.takeoff_calls > 0) {
    return mrs.fail("automatic_start resumed / disarmed / took off after a real flight without disarm -> arm");
  }

  if (mrs.output_on_calls != on_calls_before_pause) {
    return mrs.fail("automatic_start turned output ON during the pause");
  }

  mrs.state = mrs_msgs::msg::State::STATE_DISARMED;
  sleep(2.0);
  mrs.state = mrs_msgs::msg::State::STATE_ARMED; // armed again -> fresh start
  if (!mrs.waitFor([&] { return mrs.output_enabled.load(); }, 5.0)) {
    return mrs.fail("automatic_start did not start fresh after disarm -> arm");
  }

  // a short MANUAL followed by a long NO_LINK: the pilot may have flown blind, so it still needs disarm -> arm
  const int on_calls_before_blind = mrs.output_on_calls;

  mrs.state = mrs_msgs::msg::State::STATE_MANUAL;
  if (!mrs.waitFor([&] { return !mrs.output_enabled; }, 2.0)) {
    return mrs.fail("automatic_start did not turn its output off on the second MANUAL");
  }
  sleep(1.5); // MANUAL ~2 s in total, shorter than manual_resume_timeout

  mrs.state = mrs_msgs::msg::State::STATE_NO_LINK;
  sleep(5.5); // MANUAL + NO_LINK together longer than manual_resume_timeout

  mrs.state = mrs_msgs::msg::State::STATE_ARMED;
  sleep(3.0);
  if (mrs.output_enabled || mrs.output_on_calls != on_calls_before_blind) {
    return mrs.fail("automatic_start resumed after a short MANUAL followed by a long NO_LINK");
  }

  return mrs.disarm_calls == 0 && mrs.takeoff_calls == 0 ? true : mrs.fail("unexpected disarm / takeoff");
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  Tester     tester;
  const bool result = tester.test();
  tester.sleep(1.0);
  tester.reportTestResult(result);
  tester.join();
}
