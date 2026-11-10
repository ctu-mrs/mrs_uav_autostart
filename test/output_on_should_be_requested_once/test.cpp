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
  // ControlInfo shows the accepted ON only this much later, like DiagnosticsManager relaying ControlManager
  mrs.control_info_lag = 0.6; // [s], shorter than output_on_confirm_timeout (1 s)

  sleep(3.0); // DISARMED first, so the started-armed guard stays off

  mrs.state = mrs_msgs::msg::State::STATE_ARMED;
  if (!mrs.waitFor([&] { return mrs.controlInfoOutputEnabled(); }, 10.0)) {
    return mrs.fail("automatic_start never enabled output after arming -- mock setup broken");
  }

  sleep(2.0); // ControlInfo shows ON now, nothing should be requested any more

  if (mrs.output_on_calls != 1) {
    return mrs.fail("automatic_start requested output ON more than once while ControlInfo lagged");
  }

  return mrs.disarm_calls == 0 && mrs.output_off_calls == 0 && mrs.takeoff_calls == 0 ? true : mrs.fail("unexpected disarm / output OFF / takeoff");
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  Tester     tester;
  const bool result = tester.test();
  tester.sleep(1.0);
  tester.reportTestResult(result);
  tester.join();
}
