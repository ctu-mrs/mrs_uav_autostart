/* includes //{ */

#include <mrs_uav_autostart/automatic_start.hpp>

#include <algorithm>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>

#include <yaml-cpp/yaml.h>

//}

namespace mrs_uav_autostart
{

/* AutomaticStart() //{ */

AutomaticStart::AutomaticStart(rclcpp::NodeOptions options) : Node("automatic_start", options) {

  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_          = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  error_publisher_ = std::make_shared<mrs_lib::errorgraph::ErrorPublisher>(node_, clock_, "AutomaticStart", "main");

  time_zero_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  armed_since_ = time_zero_;

  data_ready_time_ = time_zero_;

  offboard_since_ = time_zero_;

  manual_since_       = time_zero_;
  armed_stable_since_ = time_zero_;
  resume_time_        = time_zero_;
  takeoff_time_       = time_zero_;

  const auto shutdown_with_error = [this](const std::string &message) {
    RCLCPP_ERROR(node_->get_logger(), "%s", message.c_str());
    error_publisher_->addOneshotError(message);
    error_publisher_->flushAndShutdown();
  };

  mrs_lib::ParamLoader param_loader(node_, "AutomaticStart");

  std::string custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);

  if (!custom_config_path.empty() && !param_loader.addYamlFile(custom_config_path)) {
    shutdown_with_error("failed to load custom_config");
  }

  if (!param_loader.addYamlFileFromParam("config_private")) {
    shutdown_with_error("failed to load config_private");
  }

  if (!param_loader.addYamlFileFromParam("config_public")) {
    shutdown_with_error("failed to load config_public");
  }

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("simulation", _simulation_);

  param_loader.loadParam("mrs_uav_autostart/main_timer_rate", _main_timer_rate_);
  param_loader.loadParam("mrs_uav_autostart/arm_to_output_timeout", _arm_to_output_timeout_);
  param_loader.loadParam("mrs_uav_autostart/takeoff_timeout", _takeoff_timeout_);
  param_loader.loadParam("mrs_uav_autostart/diagnostics_manager_timeout", _diagnostics_manager_timeout_);

  param_loader.loadParam("mrs_uav_autostart/takeoff_countdown", _takeoff_countdown_);
  param_loader.loadParam("mrs_uav_autostart/manual_resume_timeout", _manual_resume_timeout_);
  param_loader.loadParam("mrs_uav_autostart/trigger_takeoff", _trigger_takeoff_);

  if (hasObsoleteParams(custom_config_path)) {
    error_publisher_->flushAndShutdown();
  }

  if (!param_loader.loadedSuccessfully()) {
    shutdown_with_error("Could not load all parameters!");
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

  const bool is_armed = isArmed(state);

  // STATE_OFFBOARD only: armed + offboard link, no tracker active yet, i.e. on the ground
  const bool is_offboard = state == State::STATE_OFFBOARD;

  std::scoped_lock lock(mutex_uav_state_);

  // start the clocks on a rising edge, unless recovering from an ambiguous gap (see last_confirmed_armed_)
  if (is_armed && !isArmed(uav_state_) && !last_confirmed_armed_) {
    armed_since_ = clock_->now();
  }

  if (is_offboard && uav_state_ != State::STATE_OFFBOARD && !last_confirmed_offboard_) {
    offboard_since_ = clock_->now();
  }

  uav_state_ = state;

  // latch + update the confirmed-state trackers, skipping ambiguous UNKNOWN/NO_LINK readings
  if (isConfirmed(state)) {
    if (!got_confirmed_uav_state_) {
      started_armed_ = is_armed;
    }
    got_confirmed_uav_state_ = true;
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

  if (!dataReady()) {
    co_return;
  }

  auto [state, armed_since, offboard_since] = mrs_lib::get_mutexed(mutex_uav_state_, uav_state_, armed_since_, offboard_since_);
  auto control_info                         = sh_control_info_.getMsg();

  const bool armed    = isArmed(state);
  const bool offboard = state == State::STATE_OFFBOARD;

  switch (current_state_) {

  case STATE_IDLE: {

    if (isMrsFlying(state)) {
      RCLCPP_WARN(node_->get_logger(), "the UAV is already flying, nothing to start");
      co_await changeState(STATE_FINISHED);
      co_return;
    }

    if (co_await handleManualPause(state, control_info)) {
      co_return;
    }

    const auto  general_robot_info = sh_general_robot_info_.getMsg();
    const auto &preflight          = general_robot_info->preflight_status;

    if (co_await handlePossiblyInTheAir(armed, offboard, preflight)) {
      co_return;
    }

    // safe to read directly: output_enabled defaults to false whenever its source was invalid
    const bool control_output_enabled = control_info->output_enabled;

    if (co_await handleControlOutput(armed, control_output_enabled, armed_since, preflight)) {
      co_return;
    }

    if (_simulation_ && isGazeboSimulation() && !spawnerReady()) {
      co_return;
    }

    co_await handleCountdown(offboard, control_output_enabled, offboard_since);

    break;
  }

  case STATE_TAKEOFF: {

    if (control_info->flying_normally) {

      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "takeoff finished");

      co_await changeState(STATE_FINISHED);

    } else if (isTakeoffInterrupted(state)) {

      RCLCPP_WARN(node_->get_logger(), "the takeoff was interrupted, nothing left to wait for");

      co_await changeState(STATE_FINISHED);

    } else if ((clock_->now() - takeoff_time_).seconds() > _takeoff_timeout_) {

      RCLCPP_WARN(node_->get_logger(), "the takeoff did not finish within %.0f s, giving up waiting for it", _takeoff_timeout_);

      co_await changeState(STATE_FINISHED);

    } else {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "waiting for the takeoff to finish");
    }

    break;
  }

  case STATE_FINISHED: {

    // everything is done on entering FINISHED, see changeState()
    break;
  }
  }
}

//}

/* dataReady() //{ */

bool AutomaticStart::dataReady() {

  const bool got_control_info = sh_control_info_.hasMsg();
  const bool got_uav_state    = sh_uav_state_.hasMsg() && got_confirmed_uav_state_;

  // freshness-checked, so a dead DiagnosticsManager gets caught too
  const bool got_general_robot_info =
      sh_general_robot_info_.hasMsg() && (clock_->now() - sh_general_robot_info_.lastMsgTime()).seconds() <= _diagnostics_manager_timeout_;

  // position_valid is only a default until SafetyAreaManager reports (position_known); acting on it would keep
  // control output OFF and make the arm-to-output timeout disarm the UAV
  const bool got_safety_area_check = got_general_robot_info && sh_general_robot_info_.getMsg()->preflight_status.position_known;

  // all four come via DiagnosticsManager, so a missing reading is attributed to it here;
  // DiagnosticsManager's own timerErrorPublishing() attributes SafetyAreaManager specifically
  if (!got_control_info || !got_uav_state || !got_general_robot_info || !got_safety_area_check) {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *clock_, 5000,
        "waiting for data: DiagnosticsManager (control_info)=%s, DiagnosticsManager (uav_state)=%s, DiagnosticsManager (general_robot_info)=%s, "
        "DiagnosticsManager (safety_area_manager)=%s",
        got_control_info ? "true" : "FALSE", got_uav_state ? "true" : "FALSE", got_general_robot_info ? "true" : "FALSE",
        got_safety_area_check ? "true" : "FALSE");
    error_publisher_->addWaitingForNodeError({"DiagnosticsManager", "main"});

    return false;
  }

  if (data_ready_time_.nanoseconds() == 0) {
    data_ready_time_ = clock_->now();
  }

  return true;
}

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* changeState() //{ */

mrs_lib::Task<> AutomaticStart::changeState(AutostartState_t new_state) {

  RCLCPP_INFO(node_->get_logger(), "switching states %s -> %s", state_names[current_state_], state_names[new_state]);

  switch (new_state) {

  case STATE_IDLE: {

    break;
  }

  case STATE_TAKEOFF: {

    bool res = co_await takeoff();

    if (!res) {

      co_await changeState(STATE_FINISHED);

      co_return;
    }

    takeoff_time_ = clock_->now();

    break;
  }

  case STATE_FINISHED: {

    RCLCPP_INFO_ONCE(node_->get_logger(), "finished");

    timer_main_->stop();

    // the inputs are not needed any more
    sh_uav_state_.stop();
    sh_control_info_.stop();
    sh_general_robot_info_.stop();
    sh_gazebo_spawner_diag_.stop();

    break;
  }
  }

  current_state_ = new_state;
}

//}

/* handleManualPause() //{ */

// true while paused: timerMain() must not go on with the rest of IDLE
mrs_lib::Task<bool> AutomaticStart::handleManualPause(const uint8_t state, const mrs_msgs::msg::ControlInfo::ConstSharedPtr &control_info) {

  // | --- MANUAL: a pilot or the autopilot flies the UAV without offboard --- |
  // keep control output OFF so MRS can't take over (OFFBOARD finds no setpoints); a short MANUAL is an aborted
  // OFFBOARD on the ground and resumes, a long one is a real flight and the pilot must disarm and arm again

  if (state == State::STATE_MANUAL) {

    if (!manual_pause_) {
      RCLCPP_WARN(node_->get_logger(), "the UAV is flying without offboard (MANUAL), pausing");
      manual_pause_ = true;
      manual_since_ = clock_->now();
    }

    if ((clock_->now() - manual_since_).seconds() >= _manual_resume_timeout_) {
      needs_rearm_ = true;
    }

    if (we_turned_output_on_) {
      if (co_await setControlOutput(false)) {
        we_turned_output_on_           = false;
        we_turned_output_off_in_pause_ = true;
      } else {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "could not set control output OFF");
      }
    } else if (control_info->output_enabled && !we_turned_output_off_in_pause_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "control output is ON during MANUAL, but automatic start did not turn it on");
    }

    armed_stable_since_ = time_zero_;
    co_return true;
  }

  if (manual_pause_) {

    if (state == State::STATE_DISARMED) {
      RCLCPP_INFO(node_->get_logger(), "disarmed after MANUAL, starting over");
      manual_pause_                  = false;
      needs_rearm_                   = false;
      we_turned_output_off_in_pause_ = false;

    } else if (!isArmed(state)) {
      // NO_LINK / UNKNOWN: neither a disarm nor a landing -- stay paused; the UAV may still be flying manually, so a
      // long blind pause counts towards the MANUAL duration as well
      if ((clock_->now() - manual_since_).seconds() >= _manual_resume_timeout_) {
        needs_rearm_ = true;
      }
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "paused, UAV state not confirmed (NO_LINK to the autopilot, or UNKNOWN)");
      armed_stable_since_ = time_zero_;
      co_return true;

    } else if (needs_rearm_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 5000, "the UAV has flown without offboard, disarm and arm again to use automatic start");
      co_return true;

    } else {

      // the preflight heuristics lag behind a UAV that just moved (e.g. PX4's landing descent); resuming now would
      // let the "armed + possibly in the air" guard in timerMain() finish automatic start, so wait for them to settle first
      const auto  general_robot_info = sh_general_robot_info_.getMsg();
      const auto &preflight          = general_robot_info->preflight_status;

      if (state != State::STATE_OFFBOARD && !(preflight.speed_ok && preflight.height_ok && preflight.gyro_ok)) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "paused after MANUAL, waiting for the UAV to settle before resuming");
        armed_stable_since_ = time_zero_;
        co_return true;
      }

      if (armed_stable_since_.nanoseconds() == 0) {
        armed_stable_since_ = clock_->now();
      }

      if ((clock_->now() - armed_stable_since_).seconds() < 1.0) {
        co_return true;
      }

      RCLCPP_INFO(node_->get_logger(), "MANUAL was short (aborted OFFBOARD), resuming");
      manual_pause_                  = false;
      we_turned_output_off_in_pause_ = false;
      resume_time_                   = clock_->now();
    }
  }

  co_return false;
}

//}

/* handlePossiblyInTheAir() //{ */

// true when the UAV is possibly in the air (and not OFFBOARD): timerMain() must not go on with the rest of IDLE
mrs_lib::Task<bool> AutomaticStart::handlePossiblyInTheAir(const bool armed, const bool offboard, const mrs_msgs::msg::PreflightStatus &preflight) {

  const bool possibly_in_the_air = !(preflight.speed_ok && preflight.height_ok && preflight.gyro_ok);

  if (offboard || !possibly_in_the_air) {
    co_return false;
  }

  RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "preflight check failed, the UAV is possibly in the air");

  if (armed) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "the UAV is armed and possibly in the air, finishing to prevent an unwanted takeoff");

    if (we_turned_output_on_) {

      bool res = co_await setControlOutput(false);

      if (!res) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "could not set control output OFF");
      }
    }

    co_await changeState(STATE_FINISHED);
  }

  co_return true;
}

//}

/* handleControlOutput() //{ */

// true when the arm-to-output timeout disarmed the UAV and finished: timerMain() must not go on with the rest of IDLE
mrs_lib::Task<bool> AutomaticStart::handleControlOutput(const bool armed, const bool control_output_enabled, const rclcpp::Time armed_since,
                                                        const mrs_msgs::msg::PreflightStatus &preflight) {

  const bool ready_to_enable_control_output = preflight.topics_ok && preflight.position_valid;

  std_msgs::msg::Bool ready_to_enable_control_output_msg;
  ready_to_enable_control_output_msg.data = ready_to_enable_control_output;
  ph_ready_to_enable_control_output_.publish(ready_to_enable_control_output_msg);

  if (!armed || control_output_enabled) {
    co_return false;
  }

  if (ready_to_enable_control_output) {

    we_turned_output_on_ = co_await setControlOutput(true);

    if (!we_turned_output_on_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "could not set control output ON");
    }
  }

  // counted from when we could first act, so data arriving late can't make us disarm right away
  const double time_waiting = (clock_->now() - std::max({armed_since, data_ready_time_, resume_time_})).seconds();

  if (!we_turned_output_on_ && time_waiting > _arm_to_output_timeout_) {

    if (started_armed_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000,
                           "could not set control output ON for %.2f secs, but the UAV was already armed when automatic start came up, not disarming",
                           _arm_to_output_timeout_);
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "could not set control output ON for %.2f secs, disarming", _arm_to_output_timeout_);
      co_await disarm();
      co_await changeState(STATE_FINISHED);
      co_return true;
    }
  }

  co_return false;
}

//}

/* handleCountdown() //{ */

// the OFFBOARD countdown: takes off (or finishes, when not triggering the takeoff) once it runs out
mrs_lib::Task<> AutomaticStart::handleCountdown(const bool offboard, const bool control_output_enabled, const rclcpp::Time offboard_since) {

  // STATE_OFFBOARD implies armed, and offboard always follows arming, so the offboard timer alone suffices
  if (!offboard || !control_output_enabled) {
    co_return;
  }

  if (!_trigger_takeoff_) {
    co_await changeState(STATE_FINISHED);
    co_return;
  }

  const double time_in_offboard = (clock_->now() - offboard_since).seconds();

  if (time_in_offboard > _takeoff_countdown_) {
    co_await changeState(STATE_TAKEOFF);
  } else {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "taking off in %.0f", (_takeoff_countdown_ - time_in_offboard));
  }
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

/* setControlOutput() //{ */

mrs_lib::Task<bool> AutomaticStart::setControlOutput(const bool enabled) {

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "setting control output %s", enabled ? "ON" : "OFF");

  std::shared_ptr<std_srvs::srv::SetBool::Request> request = std::make_shared<std_srvs::srv::SetBool::Request>();

  request->data = enabled;

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

  if (mrs_lib::get_mutexed(mutex_uav_state_, uav_state_) == State::STATE_OFFBOARD) {

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
      {"preflight_check", "mrs_uav_managers/diagnostics_manager/preflight_check"},
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

/* spawnerReady() //{ */

bool AutomaticStart::spawnerReady(void) {

  std::scoped_lock lock(mutex_gazebo_spawner_diagnostics_);

  if (!got_gazebo_spawner_diagnostics_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "(simulation) missing spawner diagnostics");
    return false;
  }

  if (!gazebo_spawner_diagnostics_.spawn_called || gazebo_spawner_diagnostics_.processing) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "(simulation) waiting for spawner to finish spawning UAVs");
    return false;
  }

  return true;
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
