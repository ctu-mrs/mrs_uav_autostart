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

// ControlInfo shows an accepted control output ON only a moment later; until then the call is not repeated
constexpr double OUTPUT_ON_RETRY_PERIOD_S = 1.0;

// after a short MANUAL, the UAV must stay armed (and settled) this long before automatic start resumes
constexpr double ARMED_STABLE_BEFORE_RESUME_S = 1.0;

// state machine, see README.md
enum AutostartState_t
{
  STATE_WAITING_FOR_ARM,
  STATE_WAITING_FOR_OUTPUT,
  STATE_WAITING_FOR_OFFBOARD,
  STATE_COUNTDOWN,
  STATE_MANUAL_PAUSE,
  STATE_TAKEOFF,
  STATE_FINISHED
};

constexpr std::array<const char *, 7> state_names = {"WAITING_FOR_ARM", "WAITING_FOR_OUTPUT", "WAITING_FOR_OFFBOARD", "COUNTDOWN", "MANUAL_PAUSE", "TAKEOFF",
                                                     "FINISHED"};

// the steps of STATE_MANUAL_PAUSE
enum PauseStep_t
{
  PAUSE_IN_MANUAL,
  PAUSE_SETTLING,
  PAUSE_UNCONFIRMED,
  PAUSE_NEEDS_REARM
};

constexpr std::array<const char *, 4> pause_step_names = {"IN_MANUAL", "SETTLING", "UNCONFIRMED", "NEEDS_REARM"};

// the UAV as one tick of timerMain() sees it, taken once at its start; the uav_state part is written by
// callbackUavState() and copied under its mutex, everything the timer owns stays in members
struct UavSnapshot
{
  uint8_t                        state;
  bool                           armed;
  bool                           offboard;
  bool                           control_output_enabled;
  bool                           flying_normally;
  mrs_msgs::msg::PreflightStatus preflight;
  rclcpp::Time                   offboard_since;
};

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
  UavSnapshot                takeUavSnapshot();
  double                     _main_timer_rate_;

  // | ------------------------ uav state ----------------------- |

  void              callbackUavState(const mrs_msgs::msg::State::ConstSharedPtr msg);
  std::atomic<bool> got_confirmed_uav_state_ = false;
  std::mutex        mutex_uav_state_;

  // Armed at our first valid reading: we did not see the arming, so the UAV may already be in mid air, where the
  // state can't always tell (manual flight reads as ARMED when the HW API doesn't report airborne). We still proceed
  // normally, but never disarm such a UAV -- on the ground, the autopilot's own pre-takeoff auto-disarm covers it.
  std::atomic<bool> started_armed_ = false;

  // since when all DiagnosticsManager data has been available without a gap; the arm-to-output timeout and the takeoff
  // countdown never count time before it
  rclcpp::Time data_ok_since_;
  bool         data_was_ok_ = false;

  // | --------------- Gazebo spawner diagnostics --------------- |

  void                                    callbackGazeboSpawnerDiagnostics(const mrs_msgs::msg::GazeboSpawnerDiagnostics::ConstSharedPtr msg);
  std::atomic<bool>                       got_gazebo_spawner_diagnostics_ = false;
  mrs_msgs::msg::GazeboSpawnerDiagnostics gazebo_spawner_diagnostics_;
  std::mutex                              mutex_gazebo_spawner_diagnostics_;

  // | ------------------- UAV state tracking ------------------- |

  // latest reading, mrs_msgs::msg::State::STATE_*
  uint8_t uav_state_ = State::STATE_UNKNOWN;

  rclcpp::Time offboard_since_;

  // last confirmed offboard reading, so a transient UNKNOWN/NO_LINK gap can't restart the countdown
  bool last_confirmed_offboard_ = false;

  bool we_turned_output_on_ = false;

  // when toggle_output last accepted our ON, unset after a refusal; see OUTPUT_ON_RETRY_PERIOD_S
  rclcpp::Time output_on_time_;

  // | ------------------ MANUAL pause (timer-owned) ----------------- |

  PauseStep_t  pause_step_                    = PAUSE_IN_MANUAL;
  bool         we_turned_output_off_in_pause_ = false; // we turned the output OFF during this pause (ControlInfo may still lag)
  rclcpp::Time manual_since_;
  rclcpp::Time armed_stable_since_;

  rclcpp::Time takeoff_time_;

  // | -------- global checks, before the state handlers -------- |

  bool                checkDataReady();
  bool                checkAlreadyFlying(const UavSnapshot &uav);
  void                checkManual(const UavSnapshot &uav);
  mrs_lib::Task<bool> checkPossiblyInTheAir(const UavSnapshot &uav);

  // | --------------------- state handlers --------------------- |

  void                stateWaitingForArm(const UavSnapshot &uav);
  mrs_lib::Task<>     stateWaitingForOutput(const UavSnapshot &uav);
  mrs_lib::Task<>     stateCountdown(const UavSnapshot &uav);
  mrs_lib::Task<bool> stateManualPause(const UavSnapshot &uav);
  void                stateTakeoff(const UavSnapshot &uav);

  // | ---------------------- service calls --------------------- |

  mrs_lib::Task<bool> callTakeoff();
  mrs_lib::Task<bool> callControlOutput(const bool enabled);
  mrs_lib::Task<bool> callDisarm();

  // | ----------------------- simulation ----------------------- |

  bool isGazeboSimulation(void);
  bool spawnerReady(void);
  bool is_gazebo_simulation_ = false;

  // | ------------------------- params ------------------------- |

  bool hasObsoleteParams(const std::string &custom_config_path);

  bool   _trigger_takeoff_ = false;
  double _takeoff_countdown_;
  double _manual_resume_timeout_;
  double _arm_to_output_timeout_;
  double _takeoff_timeout_;
  double _diagnostics_manager_timeout_;

  // | ---------------------- state machine --------------------- |

  AutostartState_t current_state_ = STATE_WAITING_FOR_ARM;
  rclcpp::Time     state_since_; // when current_state_ was entered
  void             changeState(AutostartState_t new_state);
  void             changePauseStep(PauseStep_t new_step);

  static AutostartState_t waitingState(const UavSnapshot &uav);

  void finish(const std::string &reason);
};

//}

} // namespace mrs_uav_autostart

#endif // MRS_UAV_AUTOSTART_AUTOMATIC_START_HPP
