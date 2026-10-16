#include <rclcpp/rclcpp.hpp>

#include <mrs_msgs/msg/control_info.hpp>
#include <mrs_msgs/msg/general_robot_info.hpp>
#include <mrs_msgs/msg/state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <mrs_uav_testing/test_generic.h>

#include <atomic>

class Tester : public mrs_uav_testing::TestGeneric {

public:
  bool test(void);
};

bool Tester::test(void) {

  std::atomic<uint8_t> state{mrs_msgs::msg::State::STATE_DISARMED};
  std::atomic<bool>    output_enabled{false};
  std::atomic<bool>    moving{false};
  std::atomic<int>     output_on_calls{0}, output_off_calls{0}, disarm_calls{0}, takeoff_calls{0};

  auto pub_state = node_->create_publisher<mrs_msgs::msg::State>("/uav1/diagnostics_manager/uav_state", 10);
  auto pub_ci    = node_->create_publisher<mrs_msgs::msg::ControlInfo>("/uav1/diagnostics_manager/control_info", 10);
  auto pub_gri   = node_->create_publisher<mrs_msgs::msg::GeneralRobotInfo>("/uav1/diagnostics_manager/general_robot_info", 10);

  auto srv_output = node_->create_service<std_srvs::srv::SetBool>(
      "/uav1/control_manager/toggle_output", [&](const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res) {
        (req->data ? output_on_calls : output_off_calls)++;
        output_enabled = req->data;
        res->success   = true;
      });
  auto srv_arm = node_->create_service<std_srvs::srv::SetBool>(
      "/uav1/hw_api/arming", [&](const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res) {
        if (!req->data) {
          disarm_calls++;
        }
        res->success = true;
      });
  auto srv_takeoff = node_->create_service<std_srvs::srv::Trigger>(
      "/uav1/uav_manager/takeoff", [&](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
        takeoff_calls++;
        res->success = true;
      });

  auto timer = node_->create_wall_timer(std::chrono::milliseconds(50), [&] {
    mrs_msgs::msg::State s;
    s.stamp = node_->get_clock()->now();
    s.state = state;
    pub_state->publish(s);

    mrs_msgs::msg::ControlInfo ci;
    ci.output_enabled = output_enabled;
    pub_ci->publish(ci);

    mrs_msgs::msg::GeneralRobotInfo gri;
    gri.stamp                           = node_->get_clock()->now();
    gri.preflight_status.speed_ok       = !moving; // heuristics say "possibly in the air" while moving
    gri.preflight_status.height_ok      = !moving;
    gri.preflight_status.gyro_ok        = !moving;
    gri.preflight_status.topics_ok      = true;
    gri.preflight_status.position_valid = true;
    gri.preflight_status.position_known = true;
    pub_gri->publish(gri);
  });

  auto wait_for = [&](auto pred, double timeout) {
    const auto start = node_->get_clock()->now();
    while (rclcpp::ok() && (node_->get_clock()->now() - start).seconds() < timeout) {
      if (pred()) {
        return true;
      }
      sleep(0.05);
    }
    return false;
  };

  auto fail = [&](const std::string &what) {
    RCLCPP_ERROR(node_->get_logger(), "%s (output on %d off %d, disarm %d, takeoff %d)", what.c_str(), int(output_on_calls), int(output_off_calls),
                 int(disarm_calls), int(takeoff_calls));
    return false;
  };

  sleep(3.0); // DISARMED first, so the started-armed guard stays off

  state = mrs_msgs::msg::State::STATE_ARMED;
  if (!wait_for([&] { return output_enabled.load(); }, 10.0)) {
    return fail("automatic_start never enabled output after arming -- mock setup broken");
  }

  // | ------------------- Pausing -> Disarmed ------------------ |

  state = mrs_msgs::msg::State::STATE_MANUAL; // aborted OFFBOARD
  if (!wait_for([&] { return !output_enabled; }, 2.0)) {
    return fail("automatic_start did not turn its output off on MANUAL");
  }
  sleep(1.5); // shorter than manual_resume_timeout

  int on_calls_before = output_on_calls;

  state = mrs_msgs::msg::State::STATE_DISARMED; // the pilot disarms during the short pause
  sleep(4.5);                                   // MANUAL + DISARMED longer than manual_resume_timeout, does not matter after a disarm
  if (output_enabled || output_on_calls != on_calls_before) {
    return fail("automatic_start turned output ON while disarmed after a short MANUAL (Pausing)");
  }

  // a fresh start turns the output ON right away, a resume from the pause would wait 1 s for a stable arming first
  state = mrs_msgs::msg::State::STATE_ARMED;
  if (!wait_for([&] { return output_enabled.load(); }, 0.8)) {
    return fail("automatic_start did not start fresh after disarm -> arm during a short MANUAL (Pausing)");
  }

  // | ------------------- Settling -> Disarmed ----------------- |

  state = mrs_msgs::msg::State::STATE_MANUAL;
  if (!wait_for([&] { return !output_enabled; }, 2.0)) {
    return fail("automatic_start did not turn its output off on the second MANUAL");
  }
  moving = true; // only once MANUAL is seen: armed + possibly in the air outside a pause would finish automatic start
  sleep(1.0);

  state = mrs_msgs::msg::State::STATE_ARMED; // MANUAL ended, but the heuristics have not settled yet
  sleep(1.5);
  if (output_enabled) {
    return fail("automatic_start resumed while the UAV was still settling");
  }

  on_calls_before = output_on_calls;

  state = mrs_msgs::msg::State::STATE_DISARMED; // the pilot disarms while settling
  sleep(1.0);
  moving = false; // possibly in the air while not armed only warns
  sleep(2.0);
  if (output_enabled || output_on_calls != on_calls_before) {
    return fail("automatic_start turned output ON while disarmed after a short MANUAL (Settling)");
  }

  state = mrs_msgs::msg::State::STATE_ARMED;
  if (!wait_for([&] { return output_enabled.load(); }, 0.8)) {
    return fail("automatic_start did not start fresh after disarm -> arm during a short MANUAL (Settling)");
  }

  // | ----------------- Unconfirmed -> Disarmed ---------------- |

  state = mrs_msgs::msg::State::STATE_MANUAL;
  if (!wait_for([&] { return !output_enabled; }, 2.0)) {
    return fail("automatic_start did not turn its output off on the third MANUAL");
  }
  sleep(1.0);

  state = mrs_msgs::msg::State::STATE_NO_LINK; // still shorter than manual_resume_timeout in total
  sleep(1.5);

  on_calls_before = output_on_calls;

  state = mrs_msgs::msg::State::STATE_DISARMED; // the link is back, the pilot has disarmed
  sleep(3.0);
  if (output_enabled || output_on_calls != on_calls_before) {
    return fail("automatic_start turned output ON while disarmed after a short MANUAL (Unconfirmed)");
  }

  state = mrs_msgs::msg::State::STATE_ARMED;
  if (!wait_for([&] { return output_enabled.load(); }, 0.8)) {
    return fail("automatic_start did not start fresh after disarm -> arm during a short MANUAL (Unconfirmed)");
  }

  // | --------------------- normal takeoff --------------------- |

  state = mrs_msgs::msg::State::STATE_OFFBOARD;
  if (wait_for([&] { return takeoff_calls > 0; }, 3.0)) {
    return fail("automatic_start took off before the takeoff countdown elapsed");
  }
  if (!wait_for([&] { return takeoff_calls > 0; }, 10.0)) {
    return fail("automatic_start did not take off after the fresh start");
  }

  return disarm_calls == 0 ? true : fail("unexpected disarm");
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  Tester     tester;
  const bool result = tester.test();
  tester.sleep(1.0);
  tester.reportTestResult(result);
  tester.join();
}
