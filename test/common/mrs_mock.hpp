#pragma once

#include <rclcpp/rclcpp.hpp>

#include <mrs_msgs/msg/control_info.hpp>
#include <mrs_msgs/msg/general_robot_info.hpp>
#include <mrs_msgs/msg/state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

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
  std::atomic<double> control_info_lag{0.0};   // [s], ControlInfo shows an accepted ON only this much later
  std::atomic<bool>   refuse_output_on{false}; // toggle_output refuses an ON
  std::atomic<double> output_on_delay{0.0};    // [s], toggle_output answers an ON only this much later
  std::atomic<bool>   refuse_disarm{false};    // hw_api/arming refuses a disarm

  std::atomic<int> output_on_calls{0}, output_off_calls{0}, disarm_calls{0}, takeoff_calls{0};

  explicit MrsMock(const rclcpp::Node::SharedPtr &node) : node_(node), output_on_time_(node->get_clock()->now()) {

    pub_state_ = node_->create_publisher<mrs_msgs::msg::State>("/uav1/diagnostics_manager/uav_state", 10);
    pub_ci_    = node_->create_publisher<mrs_msgs::msg::ControlInfo>("/uav1/diagnostics_manager/control_info", 10);
    pub_gri_   = node_->create_publisher<mrs_msgs::msg::GeneralRobotInfo>("/uav1/diagnostics_manager/general_robot_info", 10);

    // answered right away, or after output_on_delay without blocking the mock (a deferred response)
    srv_output_ = node_->create_service<std_srvs::srv::SetBool>(
        "/uav1/control_manager/toggle_output",
        [this, alive = alive_](const std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> service, const std::shared_ptr<rmw_request_id_t> header,
                               const std_srvs::srv::SetBool::Request::SharedPtr req) {
          std::scoped_lock alive_lock(alive->mutex);
          if (!alive->alive) {
            return;
          }
          (req->data ? output_on_calls : output_off_calls)++;
          std_srvs::srv::SetBool::Response res;
          if (req->data && refuse_output_on) {
            res.success = false;
            res.message = "refused by the test";
          } else {
            if (req->data && !output_enabled) {
              std::scoped_lock lock(mutex_output_on_time_);
              output_on_time_ = node_->get_clock()->now();
            }
            output_enabled = req->data;
            res.success    = true;
          }
          if (req->data && output_on_delay > 0.0) {
            std::scoped_lock lock(mutex_deferred_);
            deferred_.push_back({header, res, node_->get_clock()->now() + rclcpp::Duration::from_seconds(output_on_delay)});
            return;
          }
          service->send_response(*header, res);
        });

    srv_arm_ = node_->create_service<std_srvs::srv::SetBool>(
        "/uav1/hw_api/arming", [this, alive = alive_](const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res) {
          std::scoped_lock alive_lock(alive->mutex);
          if (!alive->alive) {
            return;
          }
          if (!req->data) {
            disarm_calls++;
          }
          res->success = req->data || !refuse_disarm;
          res->message = res->success ? "" : "refused by the test";
        });

    srv_takeoff_ = node_->create_service<std_srvs::srv::Trigger>(
        "/uav1/uav_manager/takeoff", [this, alive = alive_](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
          std::scoped_lock alive_lock(alive->mutex);
          if (!alive->alive) {
            return;
          }
          takeoff_calls++;
          res->success = true;
        });

    // published on every tick once automatic start has all of DiagnosticsManager's data
    sub_ready_ = subscribe<std_msgs::msg::Bool>("/uav1/automatic_start/ready_to_enable_control_output",
                                                [this](const std_msgs::msg::Bool::ConstSharedPtr) { automatic_start_ready_ = true; });

    timer_ = node_->create_wall_timer(std::chrono::milliseconds(50), [this, alive = alive_] {
      std::scoped_lock alive_lock(alive->mutex);
      if (alive->alive) {
        publish();
      }
    });
  }

  // TestGeneric's executor keeps spinning after the test returns: a callback that is running finishes first, later ones
  // do nothing
  ~MrsMock() {
    std::scoped_lock alive_lock(alive_->mutex);
    alive_->alive = false;
  }

  MrsMock(const MrsMock &)            = delete;
  MrsMock &operator=(const MrsMock &) = delete;

  // a subscription whose callback stops with this mock, for a test that listens to automatic start; the variables the
  // callback uses must be declared before the mock, so they outlive it
  template <typename Msg>
  typename rclcpp::Subscription<Msg>::SharedPtr subscribe(const std::string &topic, std::function<void(const typename Msg::ConstSharedPtr)> callback) {
    return node_->create_subscription<Msg>(topic, 100, [alive = alive_, callback](const typename Msg::ConstSharedPtr msg) {
      std::scoped_lock alive_lock(alive->mutex);
      if (alive->alive) {
        callback(msg);
      }
    });
  }

  // waits until automatic start has all the data, including a confirmed UAV state, and a moment more: arming only after
  // this keeps the started-armed guard off however long automatic start took to come up, and automatic start does not
  // trust an arming that comes together with its data
  bool waitForAutomaticStart(const double timeout = 30.0) {
    if (!waitFor([this] { return automatic_start_ready_.load(); }, timeout)) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return true;
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
  // shared with the callbacks, which may outlive the mock
  struct Alive
  {
    std::mutex mutex;
    bool       alive = true;
  };

  std::shared_ptr<Alive>  alive_ = std::make_shared<Alive>();
  rclcpp::Node::SharedPtr node_;

  std::mutex   mutex_output_on_time_;
  rclcpp::Time output_on_time_;

  std::atomic<bool> automatic_start_ready_{false};

  // toggle_output answers waiting for output_on_delay, sent from publish()
  struct Deferred
  {
    std::shared_ptr<rmw_request_id_t> header;
    std_srvs::srv::SetBool::Response  response;
    rclcpp::Time                      due;
  };
  std::mutex            mutex_deferred_;
  std::vector<Deferred> deferred_;

  // declared last, so they are destroyed before the inputs they read
  rclcpp::Publisher<mrs_msgs::msg::State>::SharedPtr            pub_state_;
  rclcpp::Publisher<mrs_msgs::msg::ControlInfo>::SharedPtr      pub_ci_;
  rclcpp::Publisher<mrs_msgs::msg::GeneralRobotInfo>::SharedPtr pub_gri_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr            srv_output_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr            srv_arm_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr            srv_takeoff_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr          sub_ready_;
  rclcpp::TimerBase::SharedPtr                                  timer_;

  void publish() {
    {
      std::scoped_lock lock(mutex_deferred_);
      for (auto it = deferred_.begin(); it != deferred_.end();) {
        if (node_->get_clock()->now() >= it->due) {
          srv_output_->send_response(*it->header, it->response);
          it = deferred_.erase(it);
        } else {
          ++it;
        }
      }
    }

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
