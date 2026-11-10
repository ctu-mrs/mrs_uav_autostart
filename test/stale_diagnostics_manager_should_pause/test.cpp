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

  // | ------------------ waiting states stale ------------------ |

  mrs.publish_general_robot_info = false;
  sleep(3.5); // longer than diagnostics_manager_timeout (3 s)

  mrs.state = mrs_msgs::msg::State::STATE_OFFBOARD; // the countdown must not run on stale data
  sleep(7.0);                                       // longer than takeoff_countdown (5 s)
  if (mrs.takeoff_calls > 0 || mrs.disarm_calls > 0 || mrs.output_off_calls > 0) {
    return mrs.fail("automatic_start acted on the UAV while general_robot_info was stale (waiting for the takeoff)");
  }

  mrs.publish_general_robot_info = true; // DiagnosticsManager is back
  sleep(4.0);                            // the countdown restarts after the gap: no takeoff before takeoff_countdown (5 s)
  if (mrs.takeoff_calls > 0) {
    return mrs.fail("automatic_start took off right after general_robot_info was back, without a new countdown");
  }
  if (!mrs.waitFor([&] { return mrs.takeoff_calls > 0; }, 4.0)) {
    return mrs.fail("automatic_start did not take off once general_robot_info was back");
  }

  // | ----------------------- TAKEOFF stale ---------------------- |

  mrs.state = mrs_msgs::msg::State::STATE_TAKEOFF;
  sleep(1.0);

  mrs.publish_general_robot_info = false;
  sleep(3.5); // longer than diagnostics_manager_timeout (3 s)

  // would finish automatic start as an interrupted takeoff, had it not paused the TAKEOFF checks (checked in test.py)
  mrs.state = mrs_msgs::msg::State::STATE_ARMED;
  sleep(2.0);
  mrs.state = mrs_msgs::msg::State::STATE_TAKEOFF;
  sleep(1.0);

  mrs.publish_general_robot_info = true; // DiagnosticsManager is back
  sleep(1.0);

  mrs.flying_normally = true; // the takeoff finished (checked in test.py from the log)
  sleep(2.0);

  if (mrs.output_off_calls > 0 || mrs.disarm_calls > 0 || mrs.takeoff_calls != 1) {
    return mrs.fail("automatic_start acted on the UAV while general_robot_info was stale (TAKEOFF)");
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
