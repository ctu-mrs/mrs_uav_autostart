#pragma once

#include <rclcpp/rclcpp.hpp>

#include <mrs_msgs/msg/control_info.hpp>
#include <mrs_msgs/msg/general_robot_info.hpp>
#include <mrs_msgs/msg/state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

// plays DiagnosticsManager, ControlManager, HW API and UavManager for uav1's automatic start; a test sets the inputs
// below and checks the counted calls
class MrsMock {

public:
  // | --------------------- DiagnosticsManager --------------------- |

  std::atomic<uint8_t> state{mrs_msgs::msg::State::STATE_DISARMED};
  std::atomic<bool>    flying_normally{false};
  std::atomic<bool>    moving{false}; // the speed, height and gyro checks say "possibly in the air"
  std::atomic<bool>    topics_ok{true};
  std::atomic<bool>    position_valid{true};
  std::atomic<bool>    position_known{true};             // false: SafetyAreaManager has not reported yet
  std::atomic<bool>    publish_general_robot_info{true}; // false: DiagnosticsManager stopped publishing it

  // | -------------------- ControlManager, HW API ------------------- |

  std::atomic<bool>   output_enabled{false};
  std::atomic<double> control_info_lag{0.0}; // [s], ControlInfo shows an accepted ON only this much later
  std::atomic<bool>   refuse{false};         // toggle_output ON and the disarm are refused

  std::atomic<int> output_on_calls{0}, output_off_calls{0}, disarm_calls{0}, takeoff_calls{0};

  explicit MrsMock(const rclcpp::Node::SharedPtr &node) : node_(node), output_on_time_(node->get_clock()->now()) {

    pub_state_ = node_->create_publisher<mrs_msgs::msg::State>("/uav1/diagnostics_manager/uav_state", 10);
    pub_ci_    = node_->create_publisher<mrs_msgs::msg::ControlInfo>("/uav1/diagnostics_manager/control_info", 10);
    pub_gri_   = node_->create_publisher<mrs_msgs::msg::GeneralRobotInfo>("/uav1/diagnostics_manager/general_robot_info", 10);

    srv_output_ = node_->create_service<std_srvs::srv::SetBool>(
        "/uav1/control_manager/toggle_output", [this](const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res) {
          (req->data ? output_on_calls : output_off_calls)++;
          if (req->data && refuse) {
            res->success = false;
            res->message = "refused by the test";
            return;
          }
          if (req->data && !output_enabled) {
            std::scoped_lock lock(mutex_output_on_time_);
            output_on_time_ = node_->get_clock()->now();
          }
          output_enabled = req->data;
          res->success   = true;
        });

    srv_arm_ = node_->create_service<std_srvs::srv::SetBool>(
        "/uav1/hw_api/arming", [this](const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res) {
          if (!req->data) {
            disarm_calls++;
          }
          res->success = !refuse;
          res->message = refuse ? "refused by the test" : "";
        });

    srv_takeoff_ = node_->create_service<std_srvs::srv::Trigger>(
        "/uav1/uav_manager/takeoff", [this](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
          takeoff_calls++;
          res->success = true;
        });

    timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), [this] { publish(); });
  }

  // what ControlInfo shows, see control_info_lag
  bool controlInfoOutputEnabled() {
    std::scoped_lock lock(mutex_output_on_time_);
    return output_enabled && (node_->get_clock()->now() - output_on_time_).seconds() >= control_info_lag;
  }

  bool waitFor(const std::function<bool()> &pred, const double timeout) {
    const auto start = node_->get_clock()->now();
    while (rclcpp::ok() && (node_->get_clock()->now() - start).seconds() < timeout) {
      if (pred()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
  }

  // logs what failed with the call counts, returns false
  bool fail(const std::string &what) {
    RCLCPP_ERROR(node_->get_logger(), "%s (output on %d off %d, disarm %d, takeoff %d)", what.c_str(), int(output_on_calls), int(output_off_calls),
                 int(disarm_calls), int(takeoff_calls));
    return false;
  }

private:
  rclcpp::Node::SharedPtr node_;

  std::mutex   mutex_output_on_time_;
  rclcpp::Time output_on_time_;

  // declared last, so they are destroyed before the inputs they read
  rclcpp::Publisher<mrs_msgs::msg::State>::SharedPtr            pub_state_;
  rclcpp::Publisher<mrs_msgs::msg::ControlInfo>::SharedPtr      pub_ci_;
  rclcpp::Publisher<mrs_msgs::msg::GeneralRobotInfo>::SharedPtr pub_gri_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr            srv_output_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr            srv_arm_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr            srv_takeoff_;
  rclcpp::TimerBase::SharedPtr                                  timer_;

  void publish() {
    mrs_msgs::msg::State s;
    s.stamp = node_->get_clock()->now();
    s.state = state;
    pub_state_->publish(s);

    mrs_msgs::msg::ControlInfo ci;
    ci.output_enabled  = controlInfoOutputEnabled();
    ci.flying_normally = flying_normally;
    pub_ci_->publish(ci);

    if (!publish_general_robot_info) {
      return;
    }

    mrs_msgs::msg::GeneralRobotInfo gri;
    gri.stamp                           = node_->get_clock()->now();
    gri.preflight_status.speed_ok       = !moving;
    gri.preflight_status.height_ok      = !moving;
    gri.preflight_status.gyro_ok        = !moving;
    gri.preflight_status.topics_ok      = topics_ok;
    gri.preflight_status.position_valid = position_valid;
    gri.preflight_status.position_known = position_known;
    pub_gri_->publish(gri);
  }
};
