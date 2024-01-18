/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>

#include <std_msgs/Bool.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/UavManagerDiagnostics.h>
#include <mrs_msgs/ValidateReference.h>
#include <mrs_msgs/GazeboSpawnerDiagnostics.h>
#include <mrs_msgs/HwApiStatus.h>
#include <mrs_msgs/HwApiCapabilities.h>
#include <mrs_msgs/EstimationDiagnostics.h>

#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>

#include <topic_tools/shape_shifter.h>

//}

namespace mrs_uav_autostart
{

namespace automatic_start
{

/* class Topic //{ */

class Topic {
private:
  std::string topic_name_;
  ros::Time   last_time_;

public:
  Topic(std::string topic_name) : topic_name_(topic_name) {
    last_time_ = ros::Time(0);
  }

  void updateTime(void) {
    last_time_ = ros::Time::now();
  }

  ros::Time getTime(void) {
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

const char* state_names[3] = {"IDLING", "TAKEOFF", "FINISHED"};

class AutomaticStart : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle   nh_;
  std::atomic<bool> is_initialized_ = false;

  std::string _uav_name_;
  bool        _simulation_;

  // | --------------------- service clients -------------------- |

  ros::ServiceClient service_client_toggle_control_output_;
  ros::ServiceClient service_client_arm_;
  ros::ServiceClient service_client_takeoff_;
  ros::ServiceClient service_client_validate_reference_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandler<mrs_msgs::EstimationDiagnostics>     sh_estimation_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus>               sh_hw_api_status_;
  mrs_lib::SubscribeHandler<mrs_msgs::HwApiCapabilities>         sh_hw_api_capabilities_;
  mrs_lib::SubscribeHandler<sensor_msgs::Range>                  sh_distance_sensor_;
  mrs_lib::SubscribeHandler<sensor_msgs::Imu>                    sh_imu_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::UavManagerDiagnostics>     sh_uav_manager_diag_;
  mrs_lib::SubscribeHandler<mrs_msgs::GazeboSpawnerDiagnostics>  sh_gazebo_spawner_diag_;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<std_msgs::Bool> ph_can_takeoff_;

  // | ----------------------- main timer ----------------------- |

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent& event);
  double     _main_timer_rate_;

  // | ------------------- hw api diagnostics ------------------- |

  void              callbackHwApiStatus(const mrs_msgs::HwApiStatus::ConstPtr msg);
  std::atomic<bool> hw_api_connected_ = false;
  std::mutex        mutex_hw_api_status_;

  // | --------------- Gazebo spawner diagnostics --------------- |

  void                               callbackGazeboSpawnerDiagnostics(const mrs_msgs::GazeboSpawnerDiagnostics::ConstPtr msg);
  std::atomic<bool>                  got_gazebo_spawner_diagnostics = false;
  mrs_msgs::GazeboSpawnerDiagnostics gazebo_spawner_diagnostics_;
  std::mutex                         mutex_gazebo_spawner_diagnostics_;

  // | ----------------- arm and offboard check ----------------- |

  ros::Time armed_time_;
  bool      armed_ = false;

  ros::Time offboard_time_;
  bool      offboard_ = false;

  // | ------------------------ routines ------------------------ |

  bool takeoff();

  bool validateReference();

  bool toggleControlOutput(const bool& value);
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

  bool      _speed_check_enabled_ = false;
  double    _speed_check_max_speed_;
  ros::Time speed_check_violated_time_;

  // | ----------------- preflight height check ----------------- |

  bool      _height_check_enabled_ = false;
  double    _height_check_max_height_;
  ros::Time height_check_violated_time_;

  // | ----------------- preflight gyro check ----------------- |

  bool      _gyro_check_enabled_ = false;
  double    _gyro_check_max_rate_;
  ros::Time gyro_check_violated_time_;

  // | ---------------- generic topic subscribers --------------- |

  bool                     _topic_check_enabled_ = false;
  double                   _topic_check_timeout_;
  std::vector<std::string> _topic_check_topic_names_;

  std::vector<Topic>           topic_check_topics_;
  std::vector<ros::Subscriber> generic_subscriber_vec_;

  // generic callback, for any topic, to monitor its rate
  void genericCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& topic_name, const int id);
};

//}

/* onInit() //{ */

void AutomaticStart::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  armed_      = false;
  armed_time_ = ros::Time(0);

  offboard_      = false;
  offboard_time_ = ros::Time(0);

  mrs_lib::ParamLoader param_loader(nh_, "AutomaticStart");

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
    ROS_ERROR("[AutomaticStart]: Could not load all parameters!");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "AutomaticStart";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_estimation_diag_      = mrs_lib::SubscribeHandler<mrs_msgs::EstimationDiagnostics>(shopts, "estimation_diag_in");
  sh_hw_api_status_        = mrs_lib::SubscribeHandler<mrs_msgs::HwApiStatus>(shopts, "hw_api_status_in", &AutomaticStart::callbackHwApiStatus, this);
  sh_hw_api_capabilities_  = mrs_lib::SubscribeHandler<mrs_msgs::HwApiCapabilities>(shopts, "hw_api_capabilities_in");
  sh_distance_sensor_      = mrs_lib::SubscribeHandler<sensor_msgs::Range>(shopts, "distance_sensor_in");
  sh_imu_                  = mrs_lib::SubscribeHandler<sensor_msgs::Imu>(shopts, "imu_in");
  sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diagnostics_in");
  sh_uav_manager_diag_     = mrs_lib::SubscribeHandler<mrs_msgs::UavManagerDiagnostics>(shopts, "uav_manager_diagnostics_in");
  sh_gazebo_spawner_diag_  = mrs_lib::SubscribeHandler<mrs_msgs::GazeboSpawnerDiagnostics>(shopts, "gazebo_spawner_diagnostics_in",
                                                                                          &AutomaticStart::callbackGazeboSpawnerDiagnostics, this);

  // | ----------------------- publishers ----------------------- |

  ph_can_takeoff_ = mrs_lib::PublisherHandler<std_msgs::Bool>(nh_, "can_takeoff_out");

  // | --------------------- service clients -------------------- |

  service_client_takeoff_               = nh_.serviceClient<std_srvs::Trigger>("takeoff_out");
  service_client_toggle_control_output_ = nh_.serviceClient<std_srvs::SetBool>("toggle_control_output_out");
  service_client_arm_                   = nh_.serviceClient<std_srvs::SetBool>("arm_out");

  service_client_validate_reference_ = nh_.serviceClient<mrs_msgs::ValidateReference>("validate_reference_out");

  // | ------------------ setup generic topics ------------------ |

  if (_topic_check_enabled_) {

    boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback;

    for (int i = 0; i < int(_topic_check_topic_names_.size()); i++) {

      std::string topic_name = _topic_check_topic_names_[i];

      if (topic_name.at(0) != '/') {
        topic_name = "/" + _uav_name_ + "/" + topic_name;
      }

      Topic tmp_topic(topic_name);
      topic_check_topics_.push_back(tmp_topic);

      int id = i;  // id to identify which topic called the generic callback

      callback                       = [this, topic_name, id](const topic_tools::ShapeShifter::ConstPtr& msg) -> void { genericCallback(msg, topic_name, id); };
      ros::Subscriber tmp_subscriber = nh_.subscribe(topic_name, 1, callback);

      generic_subscriber_vec_.push_back(tmp_subscriber);
    }
  }

  // | --------------------- preflight check -------------------- |

  speed_check_violated_time_  = ros::Time(0);
  height_check_violated_time_ = ros::Time(0);
  gyro_check_violated_time_   = ros::Time(0);

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(_main_timer_rate_), &AutomaticStart::timerMain, this);

  is_initialized_ = true;

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* genericCallback() //{ */

void AutomaticStart::genericCallback([[maybe_unused]] const topic_tools::ShapeShifter::ConstPtr& msg, [[maybe_unused]] const std::string& topic_name,
                                     const int id) {
  topic_check_topics_[id].updateTime();
}

//}

/* callbackHwApiDiag() //{ */

void AutomaticStart::callbackHwApiStatus(const mrs_msgs::HwApiStatus::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[AutomaticStart]: getting HW API diagnostics");

  std::scoped_lock lock(mutex_hw_api_status_);

  // check armed_ state
  if (armed_ == false) {

    // if armed_ state changed to true, please "start the clock"
    if (msg->armed) {

      armed_      = true;
      armed_time_ = ros::Time::now();
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
      offboard_time_ = ros::Time::now();
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

/* callbackGazeboSpawnerDiagnostics() //{ */

void AutomaticStart::callbackGazeboSpawnerDiagnostics(const mrs_msgs::GazeboSpawnerDiagnostics::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[AutomaticStart]: getting spawner diagnostics");

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

void AutomaticStart::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_) {
    return;
  }

  bool got_uav_manager_diag     = sh_uav_manager_diag_.hasMsg();
  bool got_control_manager_diag = sh_control_manager_diag_.hasMsg();
  bool got_estimation_diag      = sh_estimation_diag_.hasMsg();
  bool got_hw_api               = sh_hw_api_status_.hasMsg() && sh_hw_api_capabilities_.hasMsg() && hw_api_connected_;

  if (!got_control_manager_diag || !got_hw_api || !got_uav_manager_diag || !got_estimation_diag) {
    ROS_WARN_THROTTLE(5.0, "[AutomaticStart]: waiting for data: ControlManager=%s, UavManager=%s, HW Api=%s, EstimationManager=%s",
                      got_control_manager_diag ? "true" : "FALSE", got_uav_manager_diag ? "true" : "FALSE", got_hw_api ? "true" : "FALSE",
                      got_estimation_diag ? "true" : "FALSE");
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

      if (possibly_in_the_air) {

        ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: preflight check failed, the UAV is possibly in the air");

        if (armed) {
          ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: -- the UAV is also armed!! shutting down to prevent unwanted system activation");
          ros::requestShutdown();
        }

        return;
      }

      // | -------------------- ready to takeoff -------------------- |

      bool   control_output_enabled = sh_control_manager_diag_.getMsg()->output_enabled;
      double time_from_arming       = (ros::Time::now() - armed_time).toSec();

      std_msgs::Bool can_takeoff_msg;
      can_takeoff_msg.data = false;

      // | -------------------- preflight checks -------------------- |

      bool position_valid = validateReference();
      bool got_topics     = topicCheck();

      bool can_takeoff = got_topics && position_valid;

      // | ---------------------------------------------------------- |

      can_takeoff_msg.data = can_takeoff;
      ph_can_takeoff_.publish(can_takeoff_msg);

      if (armed && !control_output_enabled) {

        if (can_takeoff) {

          bool res = toggleControlOutput(true);

          if (!res) {
            ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: could not set control output ON");
          }
        }

        if (time_from_arming > _control_output_timeout_) {

          ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: could not set control output ON for %.2f secs, disarming", _control_output_timeout_);
          disarm();
          changeState(STATE_FINISHED);
        }
      }

      if (_simulation_ && isGazeboSimulation()) {

        std::scoped_lock lock(mutex_gazebo_spawner_diagnostics_);

        if (got_gazebo_spawner_diagnostics) {

          if (!gazebo_spawner_diagnostics_.spawn_called || gazebo_spawner_diagnostics_.processing) {
            ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: (simulation) waiting for spawner to finish spawning UAVs");
            return;
          }

        } else {

          ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: (simulation) missing spawner diagnostics");
          return;
        }
      }

      // when armed and in offboard, takeoff
      if (armed && offboard && control_output_enabled) {

        double armed_time_diff    = (ros::Time::now() - armed_time).toSec();
        double offboard_time_diff = (ros::Time::now() - offboard_time).toSec();

        if ((armed_time_diff > _safety_timeout_) && (offboard_time_diff > _safety_timeout_)) {

          if (_handle_takeoff_) {
            changeState(STATE_TAKEOFF);
          } else {
            changeState(STATE_FINISHED);
          }

        } else {

          double min = (armed_time_diff < offboard_time_diff) ? armed_time_diff : offboard_time_diff;

          ROS_WARN_THROTTLE(1.0, "taking off in %.0f", (_safety_timeout_ - min));
        }
      }

      break;
    }

    case STATE_TAKEOFF: {

      // if takeoff finished
      if (control_manager_diagnostics->flying_normally) {

        ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: takeoff finished");

        changeState(STATE_FINISHED);

      } else {

        ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: waiting for the takeoff to finish");
      }

      break;
    }

    case STATE_FINISHED: {

      ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: finished");
      ros::requestShutdown();
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

  ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: switching states %s -> %s", state_names[current_state], state_names[new_state]);

  switch (new_state) {

    case STATE_IDLE: {

      break;
    }

    case STATE_TAKEOFF: {

      if (_pre_takeoff_sleep_ > 1.0) {
        ROS_INFO("[AutomaticStart]: sleeping for %.2f secs before takeoff", _pre_takeoff_sleep_);
        ros::Duration(_pre_takeoff_sleep_).sleep();
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

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: taking off");

  std_srvs::Trigger srv;

  bool res = service_client_takeoff_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: taking off failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for taking off failed");
  }

  return false;
}

//}

/* validateReference() //{ */

bool AutomaticStart::validateReference() {

  mrs_msgs::ValidateReference srv_out;

  srv_out.request.reference.header.frame_id = _body_frame_name_;

  bool res = service_client_validate_reference_.call(srv_out);

  if (res) {

    if (srv_out.response.success) {

      ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: current position is valid");
      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: current position is not valid (safety area, bumper)!");
      return false;
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: current position could not be validated");
    return false;
  }
}

//}

/* toggleControlOutput() //{ */

bool AutomaticStart::toggleControlOutput(const bool& value) {

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: setting control output %s", value ? "ON" : "OFF");

  std_srvs::SetBool srv;
  srv.request.data = value;

  bool res = service_client_toggle_control_output_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: setting of control output failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for toggling control output failed");
  }

  return false;
}

//}

/* disarm() //{ */

bool AutomaticStart::disarm() {

  if (!hw_api_connected_) {

    ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: cannot disarm, missing HW API status!");

    return false;
  }

  auto [armed, offboard, armed_time, offboard_time] = mrs_lib::get_mutexed(mutex_hw_api_status_, armed_, offboard_, armed_time_, offboard_time_);

  if (offboard) {

    ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: cannot disarm, already in offboard mode!");

    return false;
  }

  ROS_INFO_THROTTLE(1.0, "[AutomaticStart]: disarming");

  std_srvs::SetBool srv;
  srv.request.data = false;

  bool res = service_client_arm_.call(srv);

  if (res) {

    if (srv.response.success) {

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: disarming failed");
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[AutomaticStart]: service call for disarming failed");
  }

  return false;
}

//}

/* isGazeboSimulation() //{ */

bool AutomaticStart::isGazeboSimulation(void) {

  if (is_gazebo_simulation_) {
    return true;
  }

  ros::V_string node_list;
  ros::master::getNodes(node_list);

  for (auto& node : node_list) {
    if (node.find("mrs_drone_spawner") != std::string::npos) {
      ROS_INFO("[AutomaticStart]: MRS Gazebo Simulation detected");
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
      if ((ros::Time::now() - topic_check_topics_[i].getTime()).toSec() > _topic_check_timeout_) {
        missing_topics << std::endl << "\t" << topic_check_topics_[i].getTopicName();
        got_topics = false;
      }
    }
  }

  if (!got_topics) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[AutomaticStart]: missing data on topics: " << missing_topics.str());
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

  auto estimation_diag = sh_estimation_diag_.getMsg();

  double speed = std::hypot(estimation_diag->velocity.linear.x, estimation_diag->velocity.linear.y, estimation_diag->velocity.linear.z);

  if (speed > _speed_check_max_speed_) {
    speed_check_violated_time_ = ros::Time::now();
    ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: the estimated speed (%.2f ms^-2) is over the limit (%.2f ms^-2)", speed, _speed_check_max_speed_);
  }

  if ((ros::Time::now() - speed_check_violated_time_).toSec() < _preflight_check_time_window_) {
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

  auto capabilities = sh_hw_api_capabilities_.getMsg();

  if (!capabilities->produces_distance_sensor) {
    return true;
  }

  // | -------------------- do we have data? -------------------- |

  if (!sh_distance_sensor_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: missing distance sensor data for preflight height check");
    return false;
  }

  double height = sh_distance_sensor_.getMsg()->range;

  if (height > _height_check_max_height_) {
    height_check_violated_time_ = ros::Time::now();
    ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: the height (%.2f m) is over the limit (%.2f m)", height, _height_check_max_height_);
  }

  if ((ros::Time::now() - height_check_violated_time_).toSec() < _preflight_check_time_window_) {
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

  auto capabilities = sh_hw_api_capabilities_.getMsg();

  if (!capabilities->produces_imu) {
    return true;
  }

  // | -------------------- do we have data? -------------------- |

  if (!sh_imu_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: missing imu data for preflight gyro check");
    return false;
  }

  auto gyros = sh_imu_.getMsg()->angular_velocity;

  if (abs(gyros.x) > _gyro_check_max_rate_ || abs(gyros.y) > _gyro_check_max_rate_ || abs(gyros.z) > _gyro_check_max_rate_) {
    gyro_check_violated_time_ = ros::Time::now();
    ROS_WARN_THROTTLE(1.0, "[AutomaticStart]: the angular velocity ([%.2f, %.2f, %.2f] rad/s) is over the limit (%.2f rad/s)", gyros.x, gyros.y, gyros.z,
                      _gyro_check_max_rate_);
  }

  if ((ros::Time::now() - gyro_check_violated_time_).toSec() < _preflight_check_time_window_) {
    return false;
  } else {
    return true;
  }
}

//}

}  // namespace automatic_start

}  // namespace mrs_uav_autostart

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_uav_autostart::automatic_start::AutomaticStart, nodelet::Nodelet)
