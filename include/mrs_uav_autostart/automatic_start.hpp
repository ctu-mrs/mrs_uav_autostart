/**
 * @file automatic_start.hpp
 * @brief Automatic start: turns control output ON and triggers the takeoff once the pilot arms and switches OFFBOARD.
 *
 * Reads only DiagnosticsManager (uav_state, control_info, general_robot_info) and, in simulation, the Gazebo
 * spawner. See README.md for the state machine.
 */

#ifndef MRS_UAV_AUTOSTART_AUTOMATIC_START_HPP
#define MRS_UAV_AUTOSTART_AUTOMATIC_START_HPP

/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <array>

#include <mrs_lib/coro/task.hpp>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/errorgraph/error_publisher.h>
#include <mrs_lib/node.h>

#include <std_msgs/msg/bool.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <mrs_msgs/msg/control_info.hpp>
#include <mrs_msgs/msg/gazebo_spawner_diagnostics.hpp>
#include <mrs_msgs/msg/state.hpp>
#include <mrs_msgs/msg/general_robot_info.hpp>

#include <mrs_uav_autostart/state_checks.hpp>

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

  // zero time of the node's clock type, the "not set yet" value of the rclcpp::Time members
  rclcpp::Time time_zero_;

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

  bool dataReady();

  // | ------------------------ uav state ----------------------- |

  void              callbackUavState(const mrs_msgs::msg::State::ConstSharedPtr msg);
  std::atomic<bool> got_confirmed_uav_state_ = false;
  std::mutex        mutex_uav_state_;

  // Armed at our first valid reading: we did not see the arming, so the UAV may already be in mid air, where the
  // state can't always tell (manual flight reads as ARMED when the HW API doesn't report airborne). We still proceed
  // normally, but never disarm such a UAV -- on the ground, the autopilot's own pre-takeoff auto-disarm covers it.
  std::atomic<bool> started_armed_ = false;

  // first time all DiagnosticsManager data was available; the arm-to-output timeout never counts time before it
  rclcpp::Time data_ready_time_;

  // | --------------- Gazebo spawner diagnostics --------------- |

  void                                    callbackGazeboSpawnerDiagnostics(const mrs_msgs::msg::GazeboSpawnerDiagnostics::ConstSharedPtr msg);
  std::atomic<bool>                       got_gazebo_spawner_diagnostics_ = false;
  mrs_msgs::msg::GazeboSpawnerDiagnostics gazebo_spawner_diagnostics_;
  std::mutex                              mutex_gazebo_spawner_diagnostics_;

  // | ------------------- UAV state tracking ------------------- |

  // latest reading, mrs_msgs::msg::State::STATE_*
  uint8_t uav_state_ = State::STATE_UNKNOWN;

  rclcpp::Time armed_since_;
  rclcpp::Time offboard_since_;

  // last confirmed armed/offboard reading, so a transient UNKNOWN/NO_LINK gap can't reset the timers
  bool last_confirmed_armed_    = false;
  bool last_confirmed_offboard_ = false;

  bool we_turned_output_on_ = false;

  // | ------------------ MANUAL pause (timer-owned) ----------------- |

  bool         manual_pause_                  = false;
  bool         needs_rearm_                   = false;
  bool         we_turned_output_off_in_pause_ = false; // we turned the output OFF during this pause (ControlInfo may still lag)
  rclcpp::Time manual_since_;
  rclcpp::Time armed_stable_since_;
  rclcpp::Time resume_time_;

  rclcpp::Time takeoff_time_;

  // | ------------------------ routines ------------------------ |

  mrs_lib::Task<bool> handleManualPause(const uint8_t state, const mrs_msgs::msg::ControlInfo::ConstSharedPtr &control_info);

  mrs_lib::Task<bool> handlePossiblyInTheAir(const bool armed, const bool offboard, const mrs_msgs::msg::PreflightStatus &preflight);

  mrs_lib::Task<bool> handleControlOutput(const bool armed, const bool control_output_enabled, const rclcpp::Time armed_since,
                                          const mrs_msgs::msg::PreflightStatus &preflight);

  mrs_lib::Task<> handleCountdown(const bool offboard, const bool control_output_enabled, const rclcpp::Time offboard_since);

  mrs_lib::Task<bool> takeoff();

  mrs_lib::Task<bool> setControlOutput(const bool enabled);
  mrs_lib::Task<bool> disarm();

  bool isGazeboSimulation(void);
  bool spawnerReady(void);
  bool hasObsoleteParams(const std::string &custom_config_path);

  bool is_gazebo_simulation_ = false;

  // | ---------------------- other params ---------------------- |

  bool   _trigger_takeoff_ = false;
  double _takeoff_countdown_;
  double _manual_resume_timeout_;
  double _arm_to_output_timeout_;
  double _takeoff_timeout_;
  double _diagnostics_manager_timeout_;

  // | ---------------------- state machine --------------------- |

  AutostartState_t    current_state_ = STATE_IDLE;
  mrs_lib::Task<void> changeState(AutostartState_t new_state);
};

//}

} // namespace mrs_uav_autostart

#endif // MRS_UAV_AUTOSTART_AUTOMATIC_START_HPP
