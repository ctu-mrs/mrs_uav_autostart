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

  // | ------------------- Pausing -> Disarmed ------------------ |

  mrs.state = mrs_msgs::msg::State::STATE_MANUAL; // aborted OFFBOARD
  if (!mrs.waitFor([&] { return !mrs.output_enabled; }, 2.0)) {
    return mrs.fail("automatic_start did not turn its output off on MANUAL");
  }
  sleep(1.5); // shorter than manual_resume_timeout

  int on_calls_before = mrs.output_on_calls;

  mrs.state = mrs_msgs::msg::State::STATE_DISARMED; // the pilot disarms during the short pause
  sleep(4.5);                                       // MANUAL + DISARMED longer than manual_resume_timeout, does not matter after a disarm
  if (mrs.output_enabled || mrs.output_on_calls != on_calls_before) {
    return mrs.fail("automatic_start turned output ON while disarmed after a short MANUAL (Pausing)");
  }

  // a fresh start turns the output ON right away, a resume from the pause would wait 1 s for a stable arming first
  mrs.state = mrs_msgs::msg::State::STATE_ARMED;
  if (!mrs.waitFor([&] { return mrs.output_enabled.load(); }, 0.8)) {
    return mrs.fail("automatic_start did not start fresh after disarm -> arm during a short MANUAL (Pausing)");
  }

  // | ------------------- Settling -> Disarmed ----------------- |

  mrs.state = mrs_msgs::msg::State::STATE_MANUAL;
  if (!mrs.waitFor([&] { return !mrs.output_enabled; }, 2.0)) {
    return mrs.fail("automatic_start did not turn its output off on the second MANUAL");
  }
  mrs.moving = true; // only once MANUAL is seen: armed + possibly in the air outside a pause would finish automatic start
  sleep(1.0);

  mrs.state = mrs_msgs::msg::State::STATE_ARMED; // MANUAL ended, but the heuristics have not settled yet
  sleep(1.5);
  if (mrs.output_enabled) {
    return mrs.fail("automatic_start resumed while the UAV was still settling");
  }

  on_calls_before = mrs.output_on_calls;

  mrs.state = mrs_msgs::msg::State::STATE_DISARMED; // the pilot disarms while settling
  sleep(1.0);
  mrs.moving = false; // possibly in the air while not armed only warns
  sleep(2.0);
  if (mrs.output_enabled || mrs.output_on_calls != on_calls_before) {
    return mrs.fail("automatic_start turned output ON while disarmed after a short MANUAL (Settling)");
  }

  mrs.state = mrs_msgs::msg::State::STATE_ARMED;
  if (!mrs.waitFor([&] { return mrs.output_enabled.load(); }, 0.8)) {
    return mrs.fail("automatic_start did not start fresh after disarm -> arm during a short MANUAL (Settling)");
  }

  // | ----------------- Unconfirmed -> Disarmed ---------------- |

  mrs.state = mrs_msgs::msg::State::STATE_MANUAL;
  if (!mrs.waitFor([&] { return !mrs.output_enabled; }, 2.0)) {
    return mrs.fail("automatic_start did not turn its output off on the third MANUAL");
  }
  sleep(1.0);

  mrs.state = mrs_msgs::msg::State::STATE_NO_LINK; // still shorter than manual_resume_timeout in total
  sleep(1.5);

  on_calls_before = mrs.output_on_calls;

  mrs.state = mrs_msgs::msg::State::STATE_DISARMED; // the link is back, the pilot has disarmed
  sleep(3.0);
  if (mrs.output_enabled || mrs.output_on_calls != on_calls_before) {
    return mrs.fail("automatic_start turned output ON while disarmed after a short MANUAL (Unconfirmed)");
  }

  mrs.state = mrs_msgs::msg::State::STATE_ARMED;
  if (!mrs.waitFor([&] { return mrs.output_enabled.load(); }, 0.8)) {
    return mrs.fail("automatic_start did not start fresh after disarm -> arm during a short MANUAL (Unconfirmed)");
  }

  // | --------------------- normal takeoff --------------------- |

  mrs.state = mrs_msgs::msg::State::STATE_OFFBOARD;
  if (mrs.waitFor([&] { return mrs.takeoff_calls > 0; }, 3.0)) {
    return mrs.fail("automatic_start took off before the takeoff countdown elapsed");
  }
  if (!mrs.waitFor([&] { return mrs.takeoff_calls > 0; }, 10.0)) {
    return mrs.fail("automatic_start did not take off after the fresh start");
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
