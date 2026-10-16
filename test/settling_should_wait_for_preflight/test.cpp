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

  state = mrs_msgs::msg::State::STATE_OFFBOARD; // countdown starts
  sleep(2.0);                                   // abort before takeoff_countdown (5 s)

  state = mrs_msgs::msg::State::STATE_MANUAL; // OFFBOARD off, PX4 still "flying" until the stick is down
  if (!wait_for([&] { return !output_enabled; }, 2.0)) {
    return fail("automatic_start did not turn its output off on MANUAL");
  }
  moving = true; // only once MANUAL is seen: armed + possibly in the air outside a pause would finish automatic start
  sleep(1.5);    // shorter than manual_resume_timeout

  const int on_calls_before_settling = output_on_calls;

  state = mrs_msgs::msg::State::STATE_ARMED; // PX4 detected the landing, the heuristics still lag behind
  sleep(4.0);                                // MANUAL + settling longer than manual_resume_timeout, a confirmed ARMED does not need a rearm
  if (output_enabled || output_on_calls != on_calls_before_settling || takeoff_calls > 0) {
    return fail("automatic_start resumed / took off before the UAV settled");
  }

  moving = false; // settled
  if (wait_for([&] { return output_enabled.load(); }, 0.7)) {
    return fail("automatic_start resumed before armed was stable for 1 s");
  }
  if (!wait_for([&] { return output_enabled.load(); }, 4.0)) {
    return fail("automatic_start did not resume after the UAV settled");
  }

  state = mrs_msgs::msg::State::STATE_OFFBOARD; // retry
  if (wait_for([&] { return takeoff_calls > 0; }, 3.0)) {
    return fail("automatic_start took off too early after the OFFBOARD retry -- countdown did not restart");
  }
  if (!wait_for([&] { return takeoff_calls > 0; }, 12.0)) {
    return fail("automatic_start did not take off after the OFFBOARD retry");
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
