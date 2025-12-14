/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/node.h>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/msg/safety_area_manager_diagnostics.hpp>
#include <mrs_msgs/msg/uav_manager_diagnostics.hpp>
#include <mrs_msgs/msg/gazebo_spawner_diagnostics.hpp>
#include <mrs_msgs/msg/hw_api_status.hpp>
#include <mrs_msgs/msg/hw_api_capabilities.hpp>
#include <mrs_msgs/msg/estimation_diagnostics.hpp>

#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/imu.hpp>

//}

/* typedefs //{ */

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

//}

namespace mrs_uav_autostart
{

namespace automatic_start
{

/* class Topic //{ */

class Topic {
private:
  std::string             topic_name_;
  rclcpp::Time            last_time_;
  rclcpp::Node::SharedPtr node_;

public:
  Topic(const rclcpp::Node::SharedPtr node, std::string topic_name) : topic_name_(topic_name) {
    node_      = node;
    last_time_ = rclcpp::Time(0, 0, node->get_clock()->get_clock_type());
  }

  void updateTime(void) {
    last_time_ = node_->get_clock()->now();
  }

  rclcpp::Time getTime(void) {
    return last_time_;
  }

  std::string getTopicName(void) {
    return topic_name_;
  }
};

//}

/* class AutomaticStart //{ */

// state machine
typedef enum
{
  STATE_IDLE,
  STATE_TAKEOFF,
  STATE_FINISHED
} LandingStates_t;

const char *state_names[3] = {"IDLING", "TAKEOFF", "FINISHED"};

class AutomaticStart : public mrs_lib::Node {

public:
  AutomaticStart(rclcpp::NodeOptions options);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;

  void initialize();

  std::atomic<bool> is_initialized_ = false;

  std::string _uav_name_;
  bool        _simulation_;

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> service_client_toggle_control_output_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> service_client_arm_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> service_client_takeoff_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimationDiagnostics>        sh_estimation_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiStatus>                  sh_hw_api_status_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiCapabilities>            sh_hw_api_capabilities_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::Range>                     sh_distance_sensor_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu>                       sh_imu_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>    sh_control_manager_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::SafetyAreaManagerDiagnostics> sh_safety_area_manager_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::UavManagerDiagnostics>        sh_uav_manager_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::GazeboSpawnerDiagnostics>     sh_gazebo_spawner_diag_;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<std_msgs::msg::Bool> ph_can_takeoff_;

  // | ----------------------- main timer ----------------------- |

  std::shared_ptr<TimerType> timer_main_;
  void                       timerMain();
  double                     _main_timer_rate_;

  // | ------------------------- hw api ------------------------- |

  void              callbackHwApiStatus(const mrs_msgs::msg::HwApiStatus::ConstSharedPtr msg);
  std::atomic<bool> hw_api_connected_ = false;
  std::mutex        mutex_hw_api_status_;

  void callbackHwApiCapabilities(const mrs_msgs::msg::HwApiCapabilities::ConstSharedPtr msg);

  // | --------------- Gazebo spawner diagnostics --------------- |

  void                                    callbackGazeboSpawnerDiagnostics(const mrs_msgs::msg::GazeboSpawnerDiagnostics::ConstSharedPtr msg);
  std::atomic<bool>                       got_gazebo_spawner_diagnostics = false;
  mrs_msgs::msg::GazeboSpawnerDiagnostics gazebo_spawner_diagnostics_;
  std::mutex                              mutex_gazebo_spawner_diagnostics_;

  // | ----------------- arm and offboard check ----------------- |

  rclcpp::Time armed_time_;
  bool         armed_ = false;

  rclcpp::Time offboard_time_;
  bool         offboard_ = false;

  bool we_toggled_output_ = false;

  // | ------------------------ routines ------------------------ |

  bool takeoff();

  bool toggleControlOutput(const bool &value);
  bool disarm();

  bool isGazeboSimulation(void);
  bool topicCheck(void);
  bool preflightCheckSpeed(void);
  bool preflighCheckHeight(void);
  bool preflighCheckGyro(void);

  bool is_gazebo_simulation_ = false;

  // | ---------------------- other params ---------------------- |

  std::string _body_frame_name_;
  double      _pre_takeoff_sleep_;
  bool        _handle_takeoff_ = false;
  double      _safety_timeout_;
  double      _control_output_timeout_;

  // | ---------------------- state machine --------------------- |

  uint current_state = STATE_IDLE;
  void changeState(LandingStates_t new_state);

  // | --------------------- preflight check -------------------- |

  double _preflight_check_time_window_;

  // | ------------------ preflight speed check ----------------- |

  bool         _speed_check_enabled_ = false;
  double       _speed_check_max_speed_;
  rclcpp::Time speed_check_violated_time_;

  // | ----------------- preflight height check ----------------- |

  bool         _height_check_enabled_ = false;
  double       _height_check_max_height_;
  rclcpp::Time height_check_violated_time_;

  // | ----------------- preflight gyro check ----------------- |

  bool         _gyro_check_enabled_ = false;
  double       _gyro_check_max_rate_;
  rclcpp::Time gyro_check_violated_time_;

  // | ---------------- generic topic subscribers --------------- |

  bool                     _topic_check_enabled_ = false;
  double                   _topic_check_timeout_;
  std::vector<std::string> _topic_check_topic_names_;

  std::vector<Topic>                                  topic_check_topics_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> generic_subscriber_vec_;

  // generic callback, for any topic, to monitor its rate
  void genericCallback(std::shared_ptr<rclcpp::SerializedMessage> msg, const std::string topic, const int id);
};

//}

/* AutomaticStart() //{ */

AutomaticStart::AutomaticStart(rclcpp::NodeOptions options) : Node("automatic_start", options) {
  this->initialize();
}

//}

/* initialize() //{ */

void AutomaticStart::initialize() {

  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_subs_ = this_node().create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_sc_   = this_node().create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  armed_      = false;
  armed_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  offboard_      = false;
  offboard_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  mrs_lib::ParamLoader param_loader(this_node_ptr(), "AutomaticStart");

  std::string custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    param_loader.addYamlFile(custom_config_path);
  }

  param_loader.addYamlFileFromParam("config_private");
  param_loader.addYamlFileFromParam("config_public");

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("simulation", _simulation_);

  param_loader.loadParam("main_timer_rate", _main_timer_rate_);
  param_loader.loadParam("body_frame_name", _body_frame_name_);
  param_loader.loadParam("control_output_timeout", _control_output_timeout_);

  param_loader.loadParam("safety_timeout", _safety_timeout_);
  param_loader.loadParam("pre_takeoff_sleep", _pre_takeoff_sleep_);

  param_loader.loadParam("handle_takeoff", _handle_takeoff_);

  param_loader.loadParam("preflight_check/time_window", _preflight_check_time_window_);

  param_loader.loadParam("preflight_check/speed_check/enabled", _speed_check_enabled_);
  param_loader.loadParam("preflight_check/speed_check/max_speed", _speed_check_max_speed_);

  param_loader.loadParam("preflight_check/height_check/enabled", _height_check_enabled_);
  param_loader.loadParam("preflight_check/height_check/max_height", _height_check_max_height_);

  param_loader.loadParam("preflight_check/gyro_check/enabled", _gyro_check_enabled_);
  param_loader.loadParam("preflight_check/gyro_check/max_rate", _gyro_check_max_rate_);

  param_loader.loadParam("preflight_check/topic_check/enabled", _topic_check_enabled_);
  param_loader.loadParam("preflight_check/topic_check/timeout", _topic_check_timeout_);
  param_loader.loadParam("preflight_check/topic_check/topics", _topic_check_topic_names_);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(this_node().get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
    exit(1);
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node                                = this_node_ptr();
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;

  sh_estimation_diag_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimationDiagnostics>(shopts, "~/estimation_diag_in");
  sh_hw_api_status_   = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiStatus>(shopts, "~/hw_api_status_in", &AutomaticStart::callbackHwApiStatus, this);
  sh_hw_api_capabilities_ =
      mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiCapabilities>(shopts, "~/hw_api_capabilities_in", &AutomaticStart::callbackHwApiCapabilities, this);
  sh_distance_sensor_          = mrs_lib::SubscriberHandler<sensor_msgs::msg::Range>(shopts, "~/distance_sensor_in");
  sh_imu_                      = mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu>(shopts, "~/imu_in");
  sh_control_manager_diag_     = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>(shopts, "~/control_manager_diagnostics_in");
  sh_safety_area_manager_diag_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::SafetyAreaManagerDiagnostics>(shopts, "~/safety_area_manager_diagnostics_in");
  sh_uav_manager_diag_         = mrs_lib::SubscriberHandler<mrs_msgs::msg::UavManagerDiagnostics>(shopts, "~/uav_manager_diagnostics_in");
  sh_gazebo_spawner_diag_      = mrs_lib::SubscriberHandler<mrs_msgs::msg::GazeboSpawnerDiagnostics>(shopts, "~/gazebo_spawner_diagnostics_in",
                                                                                                     &AutomaticStart::callbackGazeboSpawnerDiagnostics, this);

  // | ----------------------- publishers ----------------------- |

  ph_can_takeoff_ = mrs_lib::PublisherHandler<std_msgs::msg::Bool>(this_node_ptr(), "~/can_takeoff_out");

  // | --------------------- service clients -------------------- |

  rclcpp::Node::SharedPtr node = this_node_ptr();

  service_client_takeoff_               = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node, "~/takeoff_out", cbkgrp_sc_);
  service_client_toggle_control_output_ = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node, "~/toggle_control_output_out", cbkgrp_sc_);
  service_client_arm_                   = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node, "~/arm_out", cbkgrp_sc_);

  // | ------------------ setup generic topics ------------------ |

  if (_topic_check_enabled_) {

    for (int i = 0; i < int(_topic_check_topic_names_.size()); i++) {

      std::string topic = _topic_check_topic_names_.at(i);

      std::string topic_name = topic.substr(0, topic.find(":"));
      std::string topic_type = topic.substr(topic.find(":") + 1, topic.length());

      if (topic_name.at(0) != '/') {
        topic_name = "/" + _uav_name_ + "/" + topic_name;
      }

      Topic tmp_topic(this_node_ptr(), topic_name);
      topic_check_topics_.push_back(tmp_topic);

      int id = i; // id to identify which topic called the generic callback

      std::function<void(std::shared_ptr<rclcpp::SerializedMessage> msg)> callback_fcn =
          std::bind(&AutomaticStart::genericCallback, this, std::placeholders::_1, topic_name, id);

      auto tmp_subscriber = this_node().create_generic_subscription(topic_name, topic_type, rclcpp::SystemDefaultsQoS(), callback_fcn);

      generic_subscriber_vec_.push_back(tmp_subscriber);
    }
  }

  // | ------------------------- timers ------------------------- |

  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node      = this_node_ptr();
  timer_opts_start.autostart = true;

  {
    std::function<void()> callback_fcn = std::bind(&AutomaticStart::timerMain, this);

    timer_main_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(_main_timer_rate_, clock_), callback_fcn);
  }

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  RCLCPP_INFO_THROTTLE(this_node().get_logger(), *clock_, 1000, "initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* genericCallback() //{ */

void AutomaticStart::genericCallback([[maybe_unused]] std::shared_ptr<rclcpp::SerializedMessage> msg, [[maybe_unused]] const std::string topic, const int id) {

  topic_check_topics_.at(id).updateTime();
}

//}

/* callbackHwApiStatus() //{ */

void AutomaticStart::callbackHwApiStatus(const mrs_msgs::msg::HwApiStatus::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(this_node().get_logger(), "getting HW API status");

  std::scoped_lock lock(mutex_hw_api_status_);

  // check armed_ state
  if (armed_ == false) {

    // if armed_ state changed to true, please "start the clock"
    if (msg->armed) {

      armed_      = true;
      armed_time_ = clock_->now();
    }

    // if we were armed_ previously
  } else if (armed_ == true) {

    // and we are not really now
    if (!msg->armed) {

      armed_ = false;
    }
  }

  // check offboard_ state
  if (offboard_ == false) {

    // if offboard_ state changed to true, please "start the clock"
    if (msg->offboard) {

      offboard_      = true;
      offboard_time_ = clock_->now();
    }

    // if we were in offboard_ previously
  } else if (offboard_ == true) {

    // and we are not really now
    if (!msg->offboard) {

      offboard_ = false;
    }
  }

  if (msg->connected) {
    hw_api_connected_ = true;
  }
}

//}

/* callbackHwApiCapabilities() //{ */

void AutomaticStart::callbackHwApiCapabilities([[maybe_unused]] const mrs_msgs::msg::HwApiCapabilities::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(this_node().get_logger(), "getting HW API capabilities");
}

//}

/* callbackGazeboSpawnerDiagnostics() //{ */

void AutomaticStart::callbackGazeboSpawnerDiagnostics(const mrs_msgs::msg::GazeboSpawnerDiagnostics::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(this_node().get_logger(), "getting spawner diagnostics");

  {
    std::scoped_lock lock(mutex_gazebo_spawner_diagnostics_);

    gazebo_spawner_diagnostics_ = *msg;

    got_gazebo_spawner_diagnostics = true;
  }
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

void AutomaticStart::timerMain() {

  if (!is_initialized_) {
    return;
  }

  bool got_uav_manager_diag         = sh_uav_manager_diag_.hasMsg();
  bool got_control_manager_diag     = sh_control_manager_diag_.hasMsg();
  bool got_safety_area_manager_diag = sh_safety_area_manager_diag_.hasMsg();
  bool got_estimation_diag          = sh_estimation_diag_.hasMsg();
  bool got_hw_api                   = sh_hw_api_status_.hasMsg() && sh_hw_api_capabilities_.hasMsg() && hw_api_connected_;

  if (!got_control_manager_diag || !got_hw_api || !got_uav_manager_diag || !got_estimation_diag || !got_safety_area_manager_diag) {
    RCLCPP_WARN_THROTTLE(this_node().get_logger(), *clock_, 5000,
                         "waiting for data: ControlManager=%s, UavManager=%s, HW "
                         "Api=%s, EstimationManager=%s , SafetyAreaManager=%s",
                         got_control_manager_diag ? "true" : "FALSE", got_uav_manager_diag ? "true" : "FALSE", got_hw_api ? "true" : "FALSE",
                         got_estimation_diag ? "true" : "FALSE", got_safety_area_manager_diag ? "true" : "FALSE");
    return;
  }

  auto [armed, offboard, armed_time, offboard_time] = mrs_lib::get_mutexed(mutex_hw_api_status_, armed_, offboard_, armed_time_, offboard_time_);
  auto control_manager_diagnostics                  = sh_control_manager_diag_.getMsg();

  switch (current_state) {

  case STATE_IDLE: {

    // | --------------------- preflight check -------------------- |

    bool speed_valid  = preflightCheckSpeed();
    bool height_valid = preflighCheckHeight();
    bool gyros_valid  = preflighCheckGyro();

    bool possibly_in_the_air = !speed_valid || !height_valid || !gyros_valid;

    if (!offboard && possibly_in_the_air) {

      RCLCPP_WARN_THROTTLE(this_node().get_logger(), *clock_, 1000, "preflight check failed, the UAV is possibly in the air");

      if (armed) {

        RCLCPP_WARN_THROTTLE(this_node().get_logger(), *clock_, 1000,
                             "-- the UAV is also armed!! finishing to prevent "
                             "unwanted system activation");

        if (we_toggled_output_) {

          bool res = toggleControlOutput(false);

          if (!res) {
            RCLCPP_WARN_THROTTLE(this_node().get_logger(), *clock_, 1000, "could not set control output OFF");
          }
        }

        changeState(STATE_FINISHED);

        return;
      }

      return;
    }

    // | -------------------- ready to takeoff -------------------- |

    bool control_output_enabled = sh_control_manager_diag_.getMsg()->output_enabled;

    std_msgs::msg::Bool can_takeoff_msg;
    can_takeoff_msg.data = false;

    // | -------------------- preflight checks -------------------- |

    bool position_valid = sh_safety_area_manager_diag_.getMsg()->position_valid_2d;
    bool got_topics     = topicCheck();

    bool can_takeoff = got_topics && position_valid;

    // | ---------------------------------------------------------- |

    can_takeoff_msg.data = can_takeoff;
    ph_can_takeoff_.publish(can_takeoff_msg);

    if (armed && !control_output_enabled) {

      if (can_takeoff) {

        bool res = toggleControlOutput(true);

        if (!res) {
          RCLCPP_WARN_THROTTLE(this_node().get_logger(), *clock_, 1000, "could not set control output ON");
        } else {
          we_toggled_output_ = true;
        }
      }

      double time_from_arming = (clock_->now() - armed_time).seconds();

      if (armed_time.seconds() > 0 && time_from_arming > _control_output_timeout_) {

        RCLCPP_WARN_THROTTLE(this_node().get_logger(), *clock_, 1000, "could not set control output ON for %.2f secs, disarming", _control_output_timeout_);
        disarm();
        changeState(STATE_FINISHED);
      }
    }

    if (_simulation_ && isGazeboSimulation()) {

      std::scoped_lock lock(mutex_gazebo_spawner_diagnostics_);

      if (got_gazebo_spawner_diagnostics) {

        if (!gazebo_spawner_diagnostics_.spawn_called || gazebo_spawner_diagnostics_.processing) {
          RCLCPP_WARN_THROTTLE(this_node().get_logger(), *clock_, 1000, "(simulation) waiting for spawner to finish spawning UAVs");
          return;
        }

      } else {

        RCLCPP_WARN_THROTTLE(this_node().get_logger(), *clock_, 1000, "(simulation) missing spawner diagnostics");
        return;
      }
    }

    // when armed and in offboard, takeoff
    if (armed && offboard && control_output_enabled) {

      if (!_handle_takeoff_) {
        changeState(STATE_FINISHED);
      } else {

        rclcpp::Duration armed_time_diff    = clock_->now() - armed_time;
        rclcpp::Duration offboard_time_diff = clock_->now() - offboard_time;

        if (armed_time_diff.seconds() > _safety_timeout_ && offboard_time_diff.seconds() > _safety_timeout_) {

          changeState(STATE_TAKEOFF);

        } else {

          double min = (armed_time_diff < offboard_time_diff) ? armed_time_diff.seconds() : offboard_time_diff.seconds();

          RCLCPP_WARN_THROTTLE(this_node().get_logger(), *clock_, 1000, "taking off in %.0f", (_safety_timeout_ - min));
        }
      }
    }

    break;
  }

  case STATE_TAKEOFF: {

    // if takeoff finished
    if (control_manager_diagnostics->flying_normally) {

      RCLCPP_INFO_THROTTLE(this_node().get_logger(), *clock_, 1000, "takeoff finished");

      changeState(STATE_FINISHED);

    } else {

      RCLCPP_WARN_THROTTLE(this_node().get_logger(), *clock_, 1000, "waiting for the takeoff to finish");
    }

    break;
  }

  case STATE_FINISHED: {

    RCLCPP_INFO_ONCE(this_node().get_logger(), "finished");

    timer_main_->stop();

    break;
  }
  }
}

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* changeState() //{ */

void AutomaticStart::changeState(LandingStates_t new_state) {

  RCLCPP_WARN_THROTTLE(this_node().get_logger(), *clock_, 1000, "switching states %s -> %s", state_names[current_state], state_names[new_state]);

  switch (new_state) {

  case STATE_IDLE: {

    break;
  }

  case STATE_TAKEOFF: {

    if (_pre_takeoff_sleep_ > 1.0) {
      RCLCPP_INFO(this_node().get_logger(), "sleeping for %.2f secs before takeoff", _pre_takeoff_sleep_);
      clock_->sleep_for(std::chrono::duration<double>(_pre_takeoff_sleep_));
    }

    bool res = takeoff();

    if (!res) {

      current_state = STATE_FINISHED;

      return;
    }

    break;
  }

  case STATE_FINISHED: {

    break;
  }

  break;
  }

  current_state = new_state;
}

//}

/* takeoff() //{ */

bool AutomaticStart::takeoff() {

  RCLCPP_INFO(this_node().get_logger(), "taking off");

  std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto response = service_client_takeoff_.callSync(request);

  if (response) {

    if (response.value()->success) {

      return true;

    } else {

      RCLCPP_ERROR_THROTTLE(this_node().get_logger(), *clock_, 1000, "taking off failed: %s", response.value()->message.c_str());
    }

  } else {

    RCLCPP_ERROR_THROTTLE(this_node().get_logger(), *clock_, 1000, "service call for taking off failed");
  }

  return false;
}

//}

/* toggleControlOutput() //{ */

bool AutomaticStart::toggleControlOutput(const bool &value) {

  RCLCPP_INFO_THROTTLE(this_node().get_logger(), *clock_, 1000, "setting control output %s", value ? "ON" : "OFF");

  std::shared_ptr<std_srvs::srv::SetBool::Request> request = std::make_shared<std_srvs::srv::SetBool::Request>();

  request->data = value;

  auto response = service_client_toggle_control_output_.callSync(request);

  if (response) {

    if (response.value()->success) {

      return true;

    } else {

      RCLCPP_ERROR_THROTTLE(this_node().get_logger(), *clock_, 1000, "setting of control output failed: %s", response.value()->message.c_str());
    }

  } else {

    RCLCPP_ERROR_THROTTLE(this_node().get_logger(), *clock_, 1000, "service call for toggling control output failed");
  }

  return false;
}

//}

/* disarm() //{ */

bool AutomaticStart::disarm() {

  if (!hw_api_connected_) {

    RCLCPP_WARN_THROTTLE(this_node().get_logger(), *clock_, 1000, "cannot disarm, missing HW API status!");

    return false;
  }

  auto [armed, offboard, armed_time, offboard_time] = mrs_lib::get_mutexed(mutex_hw_api_status_, armed_, offboard_, armed_time_, offboard_time_);

  if (offboard) {

    RCLCPP_WARN_THROTTLE(this_node().get_logger(), *clock_, 1000, "cannot disarm, already in offboard mode!");

    return false;
  }

  RCLCPP_INFO_THROTTLE(this_node().get_logger(), *clock_, 1000, "disarming");

  std::shared_ptr<std_srvs::srv::SetBool::Request> request = std::make_shared<std_srvs::srv::SetBool::Request>();

  request->data = false;

  auto response = service_client_arm_.callSync(request);

  if (response) {

    if (response.value()->success) {

      return true;

    } else {

      RCLCPP_ERROR_THROTTLE(this_node().get_logger(), *clock_, 1000, "disarming failed");
    }

  } else {

    RCLCPP_ERROR_THROTTLE(this_node().get_logger(), *clock_, 1000, "service call for disarming failed");
  }

  return false;
}

//}

/* isGazeboSimulation() //{ */

bool AutomaticStart::isGazeboSimulation(void) {

  if (is_gazebo_simulation_) {
    return true;
  }

  auto node_names = this_node().get_node_names();

  for (auto &node : node_names) {
    if (node.find("mrs_drone_spawner") != std::string::npos) {
      RCLCPP_INFO(this_node().get_logger(), "MRS Gazebo Simulation detected");
      is_gazebo_simulation_ = true;
      return true;
    }
  }

  return false;
}

//}

/* topicCheck() //{ */

bool AutomaticStart::topicCheck(void) {

  bool got_topics = true;

  std::stringstream missing_topics;

  if (_topic_check_enabled_) {

    for (int i = 0; i < int(topic_check_topics_.size()); i++) {

      if (topic_check_topics_.at(i).getTime().seconds() == 0 || (clock_->now() - topic_check_topics_.at(i).getTime()).seconds() > _topic_check_timeout_) {

        missing_topics << std::endl << "\t" << topic_check_topics_.at(i).getTopicName();
        got_topics = false;
      }
    }
  }

  if (!got_topics) {
    RCLCPP_WARN_STREAM_THROTTLE(this_node().get_logger(), *clock_, 1000, "missing data on topics: " << missing_topics.str());
  }

  return got_topics;
}

//}

// | -------- preflight cheks for detecting flyign UAV -------- |

/* preflightCheckSpeed() //{ */

bool AutomaticStart::preflightCheckSpeed(void) {

  if (!_speed_check_enabled_) {
    return true;
  }

  if (!sh_estimation_diag_.hasMsg()) {
    return false;
  }

  auto estimation_diag = sh_estimation_diag_.getMsg();

  double speed = std::hypot(estimation_diag->velocity.linear.x, estimation_diag->velocity.linear.y, estimation_diag->velocity.linear.z);

  if (speed > _speed_check_max_speed_) {
    speed_check_violated_time_ = clock_->now();
    RCLCPP_WARN_THROTTLE(this_node().get_logger(), *clock_, 1000, "the estimated speed (%.2f ms^-2) is over the limit (%.2f ms^-2)", speed,
                         _speed_check_max_speed_);
  }

  if (speed_check_violated_time_.seconds() > 0 && (clock_->now() - speed_check_violated_time_).seconds() < _preflight_check_time_window_) {
    return false;
  } else {
    return true;
  }
}

//}

/* preflighCheckHeight() //{ */

bool AutomaticStart::preflighCheckHeight(void) {

  if (!_height_check_enabled_) {
    return true;
  }

  // | ----------------- is the check possible? ----------------- |

  if (!sh_hw_api_capabilities_.hasMsg()) {
    return false;
  }

  auto capabilities = sh_hw_api_capabilities_.getMsg();

  if (!capabilities->produces_distance_sensor) {
    return true;
  }

  // | -------------------- do we have data? -------------------- |

  if (!sh_distance_sensor_.hasMsg()) {
    return true;
  }

  double height = sh_distance_sensor_.getMsg()->range;

  if (height > _height_check_max_height_) {
    height_check_violated_time_ = clock_->now();
    RCLCPP_WARN_THROTTLE(this_node().get_logger(), *clock_, 1000, "the height (%.2f m) is over the limit (%.2f m)", height, _height_check_max_height_);
  }

  if (height_check_violated_time_.seconds() > 0 && (clock_->now() - height_check_violated_time_).seconds() < _preflight_check_time_window_) {
    return false;
  } else {
    return true;
  }
}

//}

/* preflighCheckGyro() //{ */

bool AutomaticStart::preflighCheckGyro(void) {

  if (!_gyro_check_enabled_) {
    return true;
  }

  // | ----------------- is the check possible? ----------------- |

  if (!sh_hw_api_capabilities_.hasMsg()) {
    return false;
  }

  auto capabilities = sh_hw_api_capabilities_.getMsg();

  if (!capabilities->produces_imu) {
    return true;
  }

  // | -------------------- do we have data? -------------------- |

  if (!sh_imu_.hasMsg()) {
    return true;
  }

  auto gyros = sh_imu_.getMsg()->angular_velocity;

  if (abs(gyros.x) > _gyro_check_max_rate_ || abs(gyros.y) > _gyro_check_max_rate_ || abs(gyros.z) > _gyro_check_max_rate_) {
    gyro_check_violated_time_ = clock_->now();
    RCLCPP_WARN_THROTTLE(this_node().get_logger(), *clock_, 1000,
                         "the angular velocity ([%.2f, %.2f, %.2f] rad/s) is "
                         "over the limit (%.2f rad/s)",
                         gyros.x, gyros.y, gyros.z, _gyro_check_max_rate_);
  }

  if (gyro_check_violated_time_.seconds() > 0 && (clock_->now() - gyro_check_violated_time_).seconds() < _preflight_check_time_window_) {
    return false;
  } else {
    return true;
  }
}

//}

} // namespace automatic_start

} // namespace mrs_uav_autostart

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_uav_autostart::automatic_start::AutomaticStart)
