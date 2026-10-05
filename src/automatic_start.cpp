/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>

#include <mrs_lib/coro/task.hpp>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/errorgraph/error_publisher.h>
#include <mrs_lib/node.h>

#include <yaml-cpp/yaml.h>

#include <std_msgs/msg/bool.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <mrs_msgs/msg/control_info.hpp>
#include <mrs_msgs/msg/gazebo_spawner_diagnostics.hpp>
#include <mrs_msgs/msg/state.hpp>
#include <mrs_msgs/msg/general_robot_info.hpp>

//}

/* typedefs //{ */

#if USE_ROS_TIMER == 1
using TimerType = mrs_lib::ROSTimer;
#else
using TimerType = mrs_lib::ThreadTimer;
#endif

//}

namespace mrs_uav_autostart
{

/* class AutomaticStart //{ */

// state machine
enum AutostartState_t
{
  STATE_IDLE,
  STATE_TAKEOFF,
  STATE_FINISHED
};

constexpr std::array<const char *, 3> state_names = {"IDLE", "TAKEOFF", "FINISHED"};

class AutomaticStart : public mrs_lib::Node {

public:
  AutomaticStart(rclcpp::NodeOptions options);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_;

  std::atomic<bool> is_initialized_ = false;

  std::string _uav_name_;
  bool        _simulation_;

  std::shared_ptr<mrs_lib::errorgraph::ErrorPublisher> error_publisher_;

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> service_client_toggle_control_output_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> service_client_arm_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> service_client_takeoff_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandler<mrs_msgs::msg::State>                    sh_uav_state_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlInfo>              sh_control_info_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::GazeboSpawnerDiagnostics> sh_gazebo_spawner_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::GeneralRobotInfo>         sh_general_robot_info_;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<std_msgs::msg::Bool> ph_ready_to_enable_control_output_;

  // | ----------------------- main timer ----------------------- |

  std::shared_ptr<TimerType> timer_main_;
  mrs_lib::Task<>            timerMain();
  double                     _main_timer_rate_;

  // | ------------------------ uav state ----------------------- |

  void              callbackUavState(const mrs_msgs::msg::State::ConstSharedPtr msg);
  std::atomic<bool> uav_state_valid_ever_ = false;
  std::mutex        mutex_uav_state_;

  // Armed at our first valid reading: we did not see the arming, so the UAV may already be in mid air, where the
  // state can't tell (manual flight reads as ARMED). We still proceed normally, but never disarm such a UAV --
  // on the ground, the autopilot's own pre-takeoff auto-disarm covers it.
  std::atomic<bool> started_armed_ = false;

  // first time all DiagnosticsManager data was available; the arm-to-output timeout never counts time before it
  rclcpp::Time data_ready_time_;

  // | --------------- Gazebo spawner diagnostics --------------- |

  void                                    callbackGazeboSpawnerDiagnostics(const mrs_msgs::msg::GazeboSpawnerDiagnostics::ConstSharedPtr msg);
  std::atomic<bool>                       got_gazebo_spawner_diagnostics_ = false;
  mrs_msgs::msg::GazeboSpawnerDiagnostics gazebo_spawner_diagnostics_;
  std::mutex                              mutex_gazebo_spawner_diagnostics_;

  // | ----------------- arm and offboard check ----------------- |

  rclcpp::Time armed_time_;
  bool         armed_ = false;

  rclcpp::Time offboard_time_;
  bool         offboard_ = false;

  // a tracker (or a pilot) is already flying the UAV, so there is nothing left for us to start
  bool flying_ = false;

  // last confirmed armed/offboard reading, so a transient UNKNOWN/NO_LINK gap can't reset the timers
  bool last_confirmed_armed_    = false;
  bool last_confirmed_offboard_ = false;

  // MANUAL: armed, not offboard, the autopilot reports in-air -- it flies the UAV without offboard
  bool manual_ = false;

  // explicit STATE_DISARMED only; NO_LINK/UNKNOWN are not a disarm
  bool disarmed_ = false;

  bool we_toggled_output_ = false;

  // | ------------------ MANUAL pause (timer-owned) ----------------- |

  bool         manual_pause_      = false;
  bool         needs_rearm_       = false;
  bool         paused_output_off_ = false; // we turned the output OFF during this pause (ControlInfo may still lag)
  rclcpp::Time manual_since_;
  rclcpp::Time armed_stable_since_;
  rclcpp::Time resume_time_;

  // | ------------------------ routines ------------------------ |

  mrs_lib::Task<bool> takeoff();

  mrs_lib::Task<bool> toggleControlOutput(const bool &value);
  mrs_lib::Task<bool> disarm();

  bool isGazeboSimulation(void);
  bool hasObsoleteParams(const std::string &custom_config_path);

  bool is_gazebo_simulation_ = false;

  // | ---------------------- other params ---------------------- |

  bool   _trigger_takeoff_ = false;
  double _takeoff_countdown_;
  double _manual_abort_max_duration_;
  double _arm_to_output_timeout_;
  double _diagnostics_manager_timeout_;

  // | ---------------------- state machine --------------------- |

  AutostartState_t    current_state_ = STATE_IDLE;
  mrs_lib::Task<void> changeState(AutostartState_t new_state);
};

//}

/* AutomaticStart() //{ */

AutomaticStart::AutomaticStart(rclcpp::NodeOptions options) : Node("automatic_start", options) {

  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_          = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  error_publisher_ = std::make_shared<mrs_lib::errorgraph::ErrorPublisher>(node_, clock_, "AutomaticStart", "main");

  armed_      = false;
  armed_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  data_ready_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  offboard_      = false;
  offboard_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  manual_since_       = rclcpp::Time(0, 0, clock_->get_clock_type());
  armed_stable_since_ = rclcpp::Time(0, 0, clock_->get_clock_type());
  resume_time_        = rclcpp::Time(0, 0, clock_->get_clock_type());

  mrs_lib::ParamLoader param_loader(node_, "AutomaticStart");

  std::string custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    if (!param_loader.addYamlFile(custom_config_path)) {
      RCLCPP_ERROR(node_->get_logger(), "failed to load custom_config");
      error_publisher_->addOneshotError("failed to load custom_config");
      error_publisher_->flushAndShutdown();
    }
  }

  if (!param_loader.addYamlFileFromParam("config_private")) {
    RCLCPP_ERROR(node_->get_logger(), "failed to load config_private");
    error_publisher_->addOneshotError("failed to load config_private");
    error_publisher_->flushAndShutdown();
  }

  if (!param_loader.addYamlFileFromParam("config_public")) {
    RCLCPP_ERROR(node_->get_logger(), "failed to load config_public");
    error_publisher_->addOneshotError("failed to load config_public");
    error_publisher_->flushAndShutdown();
  }

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("simulation", _simulation_);

  param_loader.loadParam("mrs_uav_autostart/main_timer_rate", _main_timer_rate_);
  param_loader.loadParam("mrs_uav_autostart/arm_to_output_timeout", _arm_to_output_timeout_);
  param_loader.loadParam("mrs_uav_autostart/diagnostics_manager_timeout", _diagnostics_manager_timeout_);

  param_loader.loadParam("mrs_uav_autostart/takeoff_countdown", _takeoff_countdown_);
  param_loader.loadParam("mrs_uav_autostart/manual_abort_max_duration", _manual_abort_max_duration_);
  param_loader.loadParam("mrs_uav_autostart/trigger_takeoff", _trigger_takeoff_);

  if (hasObsoleteParams(custom_config_path)) {
    error_publisher_->flushAndShutdown();
  }

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(this_node().get_logger(), "Could not load all parameters!");
    error_publisher_->addOneshotError("Could not load all parameters!");
    error_publisher_->flushAndShutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node                                = node_;
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_;

  sh_uav_state_           = mrs_lib::SubscriberHandler<mrs_msgs::msg::State>(shopts, "~/uav_state_in", &AutomaticStart::callbackUavState, this);
  sh_control_info_        = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlInfo>(shopts, "~/control_info_in");
  sh_gazebo_spawner_diag_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::GazeboSpawnerDiagnostics>(shopts, "~/gazebo_spawner_diagnostics_in",
                                                                                                &AutomaticStart::callbackGazeboSpawnerDiagnostics, this);
  sh_general_robot_info_  = mrs_lib::SubscriberHandler<mrs_msgs::msg::GeneralRobotInfo>(shopts, "~/general_robot_info_in");

  // | ----------------------- publishers ----------------------- |

  ph_ready_to_enable_control_output_ = mrs_lib::PublisherHandler<std_msgs::msg::Bool>(node_, "~/ready_to_enable_control_output_out");

  // | --------------------- service clients -------------------- |

  service_client_takeoff_               = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/takeoff_out", cbkgrp_);
  service_client_toggle_control_output_ = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "~/toggle_control_output_out", cbkgrp_);
  service_client_arm_                   = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "~/arm_out", cbkgrp_);

  // | ------------------------- timers ------------------------- |

  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node           = node_;
  timer_opts_start.autostart      = true;
  timer_opts_start.callback_group = cbkgrp_;

  timer_main_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(_main_timer_rate_, clock_), &AutomaticStart::timerMain, this);

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackUavState() //{ */

void AutomaticStart::callbackUavState(const mrs_msgs::msg::State::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting UAV state");

  const uint8_t state = msg->state;

  // DISARMED/NO_LINK/UNKNOWN mean "not confidently armed"
  const bool is_armed =
      !(state == mrs_msgs::msg::State::STATE_DISARMED || state == mrs_msgs::msg::State::STATE_NO_LINK || state == mrs_msgs::msg::State::STATE_UNKNOWN);

  // STATE_OFFBOARD means armed + offboard link, no tracker active yet -- i.e. on the ground.
  // Any other flying state must count as not offboard, so timerMain()'s possibly_in_the_air guard catches it.
  const bool is_offboard = state == mrs_msgs::msg::State::STATE_OFFBOARD;

  std::scoped_lock lock(mutex_uav_state_);

  // start the clocks on a rising edge, unless recovering from an ambiguous gap (see last_confirmed_armed_)
  if (is_armed && !armed_ && !last_confirmed_armed_) {
    armed_time_ = clock_->now();
  }

  if (is_offboard && !offboard_ && !last_confirmed_offboard_) {
    offboard_time_ = clock_->now();
  }

  armed_    = is_armed;
  offboard_ = is_offboard;
  flying_   = state == mrs_msgs::msg::State::STATE_TAKEOFF || state == mrs_msgs::msg::State::STATE_HOVER || state == mrs_msgs::msg::State::STATE_GOTO ||
            state == mrs_msgs::msg::State::STATE_TRAJECTORY || state == mrs_msgs::msg::State::STATE_LAND || state == mrs_msgs::msg::State::STATE_RC_MODE ||
            state == mrs_msgs::msg::State::STATE_MIDAIR || state == mrs_msgs::msg::State::STATE_EHOVER || state == mrs_msgs::msg::State::STATE_ELAND ||
            state == mrs_msgs::msg::State::STATE_FAILSAFE;
  manual_   = state == mrs_msgs::msg::State::STATE_MANUAL;
  disarmed_ = state == mrs_msgs::msg::State::STATE_DISARMED;

  // latch + update the confirmed-state trackers, skipping ambiguous UNKNOWN/NO_LINK readings
  if (state != mrs_msgs::msg::State::STATE_NO_LINK && state != mrs_msgs::msg::State::STATE_UNKNOWN) {
    if (!uav_state_valid_ever_) {
      started_armed_ = is_armed;
    }
    uav_state_valid_ever_    = true;
    last_confirmed_armed_    = is_armed;
    last_confirmed_offboard_ = is_offboard;
  }
}

//}

/* callbackGazeboSpawnerDiagnostics() //{ */

void AutomaticStart::callbackGazeboSpawnerDiagnostics(const mrs_msgs::msg::GazeboSpawnerDiagnostics::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting spawner diagnostics");

  {
    std::scoped_lock lock(mutex_gazebo_spawner_diagnostics_);

    gazebo_spawner_diagnostics_ = *msg;

    got_gazebo_spawner_diagnostics_ = true;
  }
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

mrs_lib::Task<> AutomaticStart::timerMain() {

  if (!is_initialized_) {
    co_return;
  }

  bool got_control_info = sh_control_info_.hasMsg();
  bool got_uav_state    = sh_uav_state_.hasMsg() && uav_state_valid_ever_;

  // freshness-checked, so a dead DiagnosticsManager gets caught too
  bool got_general_robot_info =
      sh_general_robot_info_.hasMsg() && (clock_->now() - sh_general_robot_info_.lastMsgTime()).seconds() <= _diagnostics_manager_timeout_;

  // position_known guards against reading a not-yet-reported position_valid as a confirmed violation
  bool got_safety_area_manager = got_general_robot_info && sh_general_robot_info_.getMsg()->preflight_status.position_known;

  // all four come via DiagnosticsManager, so a missing reading is attributed to it here;
  // DiagnosticsManager's own timerErrorPublishing() attributes SafetyAreaManager specifically
  if (!got_control_info || !got_uav_state || !got_general_robot_info || !got_safety_area_manager) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *clock_, 5000,
        "waiting for data: DiagnosticsManager (control_info)=%s, DiagnosticsManager (uav_state)=%s, DiagnosticsManager (general_robot_info)=%s, "
        "DiagnosticsManager (safety_area_manager)=%s",
        got_control_info ? "true" : "FALSE", got_uav_state ? "true" : "FALSE", got_general_robot_info ? "true" : "FALSE",
        got_safety_area_manager ? "true" : "FALSE");
    error_publisher_->addWaitingForNodeError({"DiagnosticsManager", "main"});

    co_return;
  }

  if (data_ready_time_.nanoseconds() == 0) {
    data_ready_time_ = clock_->now();
  }

  auto [armed, offboard, flying, manual, disarmed, armed_time, offboard_time] =
      mrs_lib::get_mutexed(mutex_uav_state_, armed_, offboard_, flying_, manual_, disarmed_, armed_time_, offboard_time_);
  auto control_info = sh_control_info_.getMsg();

  switch (current_state_) {

  case STATE_IDLE: {

    if (flying) {
      RCLCPP_WARN(node_->get_logger(), "the UAV is already flying, nothing to start");
      co_await changeState(STATE_FINISHED);
      co_return;
    }

    // | ------ MANUAL: PX4 flies the UAV without offboard ------ |
    // keep control output OFF so MRS can't take over (OFFBOARD finds no setpoints); a short MANUAL is an aborted
    // OFFBOARD on the ground and resumes, a long one is a real flight and needs disarm -> arm

    if (manual) {

      if (!manual_pause_) {
        RCLCPP_WARN(node_->get_logger(), "the UAV is flying without offboard (MANUAL), pausing");
        manual_pause_ = true;
        manual_since_ = clock_->now();
      }

      if ((clock_->now() - manual_since_).seconds() >= _manual_abort_max_duration_) {
        needs_rearm_ = true;
      }

      if (we_toggled_output_) {
        if (co_await toggleControlOutput(false)) {
          we_toggled_output_ = false;
          paused_output_off_ = true;
        } else {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "could not set control output OFF");
        }
      } else if (control_info->output_enabled && !paused_output_off_) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "control output is ON during MANUAL, but automatic start did not turn it on");
      }

      armed_stable_since_ = rclcpp::Time(0, 0, clock_->get_clock_type());
      co_return;
    }

    if (manual_pause_) {

      if (disarmed) {
        RCLCPP_INFO(node_->get_logger(), "disarmed after MANUAL, starting over");
        manual_pause_      = false;
        needs_rearm_       = false;
        paused_output_off_ = false;

      } else if (!armed) {
        // NO_LINK / UNKNOWN: neither a disarm nor a landing -- stay paused; the UAV may still be flying manually, so a
        // long blind pause counts towards the MANUAL duration as well
        if ((clock_->now() - manual_since_).seconds() >= _manual_abort_max_duration_) {
          needs_rearm_ = true;
        }
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "paused, UAV state not confirmed (NO_LINK to the autopilot, or UNKNOWN)");
        armed_stable_since_ = rclcpp::Time(0, 0, clock_->get_clock_type());
        co_return;

      } else if (needs_rearm_) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 5000, "the UAV has flown without offboard, disarm and arm again to use automatic start");
        co_return;

      } else {

        // the preflight heuristics lag behind a UAV that just moved (e.g. PX4's landing descent); resuming now would
        // let the "armed + possibly in the air" guard below finish automatic start, so wait for them to settle first
        const auto &preflight_pause = sh_general_robot_info_.getMsg()->preflight_status;

        if (!offboard && !(preflight_pause.speed_ok && preflight_pause.height_ok && preflight_pause.gyro_ok)) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "paused after MANUAL, waiting for the UAV to settle before resuming");
          armed_stable_since_ = rclcpp::Time(0, 0, clock_->get_clock_type());
          co_return;
        }

        if (armed_stable_since_.nanoseconds() == 0) {
          armed_stable_since_ = clock_->now();
        }

        if ((clock_->now() - armed_stable_since_).seconds() < 1.0) {
          co_return;
        }

        RCLCPP_INFO(node_->get_logger(), "MANUAL was short (aborted OFFBOARD), resuming");
        manual_pause_      = false;
        paused_output_off_ = false;
        resume_time_       = clock_->now();
      }
    }

    // | --------------------- preflight check -------------------- |

    const auto &preflight = sh_general_robot_info_.getMsg()->preflight_status;

    bool possibly_in_the_air = !(preflight.speed_ok && preflight.height_ok && preflight.gyro_ok);

    if (!offboard && possibly_in_the_air) {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "preflight check failed, the UAV is possibly in the air");

      if (armed) {

        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000,
                             "-- the UAV is also armed!! finishing to prevent "
                             "unwanted system activation");

        if (we_toggled_output_) {

          bool res = co_await toggleControlOutput(false);

          if (!res) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "could not set control output OFF");
          }
        }

        co_await changeState(STATE_FINISHED);
      }

      co_return;
    }

    // | -------------------- ready to takeoff -------------------- |

    // safe to read directly: output_enabled defaults to false whenever its source was invalid
    const bool control_output_enabled         = control_info->output_enabled;
    const bool ready_to_enable_control_output = preflight.topics_ok && preflight.position_valid;

    std_msgs::msg::Bool ready_to_enable_control_output_msg;
    ready_to_enable_control_output_msg.data = ready_to_enable_control_output;
    ph_ready_to_enable_control_output_.publish(ready_to_enable_control_output_msg);

    if (armed && !control_output_enabled) {

      if (ready_to_enable_control_output) {

        we_toggled_output_ = co_await toggleControlOutput(true);

        if (!we_toggled_output_) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "could not set control output ON");
        }
      }

      // counted from when we could first act, so data arriving late can't make us disarm right away
      const double time_waiting = (clock_->now() - std::max({armed_time, data_ready_time_, resume_time_})).seconds();

      if (!we_toggled_output_ && time_waiting > _arm_to_output_timeout_) {

        if (started_armed_) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000,
                               "could not set control output ON for %.2f secs, but the UAV was already armed when automatic start came up, not disarming",
                               _arm_to_output_timeout_);
        } else {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "could not set control output ON for %.2f secs, disarming", _arm_to_output_timeout_);
          co_await disarm();
          co_await changeState(STATE_FINISHED);
          co_return;
        }
      }
    }

    if (_simulation_ && isGazeboSimulation()) {

      std::scoped_lock lock(mutex_gazebo_spawner_diagnostics_);

      if (got_gazebo_spawner_diagnostics_) {

        if (!gazebo_spawner_diagnostics_.spawn_called || gazebo_spawner_diagnostics_.processing) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "(simulation) waiting for spawner to finish spawning UAVs");
          co_return;
        }

      } else {

        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "(simulation) missing spawner diagnostics");
        co_return;
      }
    }

    // STATE_OFFBOARD implies armed, and offboard always follows arming, so the offboard timer alone suffices
    if (offboard && control_output_enabled) {

      if (!_trigger_takeoff_) {
        co_await changeState(STATE_FINISHED);
      } else {

        const double offboard_time_diff = (clock_->now() - offboard_time).seconds();

        if (offboard_time_diff > _takeoff_countdown_) {
          co_await changeState(STATE_TAKEOFF);
        } else {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "taking off in %.0f", (_takeoff_countdown_ - offboard_time_diff));
        }
      }
    }

    break;
  }

  case STATE_TAKEOFF: {

    // if takeoff finished
    if (control_info->flying_normally) {

      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "takeoff finished");

      co_await changeState(STATE_FINISHED);

    } else {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "waiting for the takeoff to finish");
    }

    break;
  }

  case STATE_FINISHED: {

    RCLCPP_INFO_ONCE(node_->get_logger(), "finished");

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

mrs_lib::Task<> AutomaticStart::changeState(AutostartState_t new_state) {

  RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "switching states %s -> %s", state_names[current_state_], state_names[new_state]);

  switch (new_state) {

  case STATE_IDLE: {

    break;
  }

  case STATE_TAKEOFF: {

    bool res = co_await takeoff();

    if (!res) {

      current_state_ = STATE_FINISHED;

      co_return;
    }

    break;
  }

  case STATE_FINISHED: {

    break;
  }
  }

  current_state_ = new_state;
}

//}

/* takeoff() //{ */

mrs_lib::Task<bool> AutomaticStart::takeoff() {

  RCLCPP_INFO(node_->get_logger(), "taking off");

  std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto response = co_await service_client_takeoff_.callAwaitable(request);

  if (response) {

    if (response.value()->success) {

      co_return true;

    } else {

      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "taking off failed: %s", response.value()->message.c_str());
    }

  } else {

    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for taking off failed");
  }

  co_return false;
}

//}

/* toggleControlOutput() //{ */

mrs_lib::Task<bool> AutomaticStart::toggleControlOutput(const bool &value) {

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "setting control output %s", value ? "ON" : "OFF");

  std::shared_ptr<std_srvs::srv::SetBool::Request> request = std::make_shared<std_srvs::srv::SetBool::Request>();

  request->data = value;

  auto response = co_await service_client_toggle_control_output_.callAwaitable(request);

  if (response) {

    if (response.value()->success) {

      co_return true;

    } else {

      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "setting of control output failed: %s", response.value()->message.c_str());
    }

  } else {

    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for toggling control output failed");
  }

  co_return false;
}

//}

/* disarm() //{ */

mrs_lib::Task<bool> AutomaticStart::disarm() {

  if (mrs_lib::get_mutexed(mutex_uav_state_, offboard_)) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "cannot disarm, already in offboard mode!");

    co_return false;
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "disarming");

  std::shared_ptr<std_srvs::srv::SetBool::Request> request = std::make_shared<std_srvs::srv::SetBool::Request>();

  request->data = false;

  auto response = co_await service_client_arm_.callAwaitable(request);

  if (response) {

    if (response.value()->success) {

      co_return true;

    } else {

      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "disarming failed");
    }

  } else {

    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for disarming failed");
  }

  co_return false;
}

//}

/* hasObsoleteParams() //{ */

// An obsolete key would otherwise be silently ignored and fall back to the default -- e.g. an old
// "handle_takeoff: false" would make us take off on our own. Refuse to start instead.
bool AutomaticStart::hasObsoleteParams(const std::string &custom_config_path) {

  if (custom_config_path.empty()) {
    return false;
  }

  // old name -> new name
  const std::vector<std::pair<std::string, std::string>> obsolete_params = {
      {"safety_timeout", "mrs_uav_autostart/takeoff_countdown"},
      {"handle_takeoff", "mrs_uav_autostart/trigger_takeoff"},
      {"control_output_timeout", "mrs_uav_autostart/arm_to_output_timeout"},
      {"pre_takeoff_sleep", "mrs_uav_autostart/takeoff_countdown"},
  };

  const YAML::Node config = YAML::LoadFile(custom_config_path);

  if (!config.IsMap()) {
    return false;
  }

  // a missing key gives an invalid node: test it with operator bool before calling IsMap() on it
  const YAML::Node section = config["mrs_uav_autostart"];

  bool found = false;

  const auto report = [&](const std::string &name, const std::string &new_name) {
    RCLCPP_ERROR(node_->get_logger(), "obsolete parameter '%s' found in custom_config, use '%s' instead", name.c_str(), new_name.c_str());
    error_publisher_->addOneshotError("obsolete parameter '" + name + "' in custom_config");
    found = true;
  };

  for (const auto &[old_name, new_name] : obsolete_params) {

    if (config[old_name]) {
      report(old_name, new_name);
    }

    if (section && section.IsMap() && section[old_name]) {
      report("mrs_uav_autostart/" + old_name, new_name);
    }
  }

  return found;
}

//}

/* isGazeboSimulation() //{ */

bool AutomaticStart::isGazeboSimulation(void) {

  if (is_gazebo_simulation_) {
    return true;
  }

  for (auto &node : node_->get_node_names()) {
    if (node.find("mrs_drone_spawner") != std::string::npos) {
      RCLCPP_INFO(node_->get_logger(), "MRS Gazebo Simulation detected");
      is_gazebo_simulation_ = true;
      return true;
    }
  }

  return false;
}

//}

} // namespace mrs_uav_autostart

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_uav_autostart::AutomaticStart)
