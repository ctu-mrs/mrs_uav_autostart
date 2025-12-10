#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_lib/timer_handler.h>

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

#include <mrs_uav_testing/test_generic.h>

using namespace std::chrono_literals;

#include <mrs_msgs/msg/hw_api_velocity_hdg_cmd.hpp>

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester();

  mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiVelocityHdgCmd> ph_velocity_;

  std::shared_ptr<TimerType> timer_main_;
  void                       timerMain();

  std::string uav_name = "uav1";

  bool test(void);
};

Tester::Tester() : mrs_uav_testing::TestGeneric() {

  ph_velocity_ = mrs_lib::PublisherHandler<mrs_msgs::msg::HwApiVelocityHdgCmd>(node_, "/multirotor_simulator/uav1/velocity_hdg_cmd");

  std::function<void()> callback_fcn = std::bind(&Tester::timerMain, this);

  mrs_lib::TimerHandlerOptions opts;
  opts.autostart = false;
  opts.node      = node_;

  timer_main_ = std::make_shared<TimerType>(opts, rclcpp::Rate(100.0, clock_), callback_fcn);
}

void Tester::timerMain() {

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "timerMain() spinning, the drone should be moving");

  mrs_msgs::msg::HwApiVelocityHdgCmd msg;

  msg.velocity.x = 1.0;
  msg.velocity.z = 1.5;

  ph_velocity_.publish(msg);
}

bool Tester::test(void) {

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh;

  {
    auto [uhopt, message] = getUAVHandler(uav_name);

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed obtain handler for '%s': '%s'", uav_name.c_str(), message.c_str());
      return false;
    }

    uh = uhopt.value();
  }

  {
    while (rclcpp::ok()) {
      if (uh->mrsSystemReady()) {
        break;
      }
    }
  }

  sleep(1.0);

  RCLCPP_INFO(node_->get_logger(), "[%s]: arming the drone", name_.c_str());

  {

    std::shared_ptr<std_srvs::srv::SetBool::Request> request = std::make_shared<std_srvs::srv::SetBool::Request>();

    request->data = true;

    {
      auto response = uh->sch_arming_.callSync(request);

      if (!response || !response.value()->success) {
        RCLCPP_ERROR(node_->get_logger(), "could not arm the drone");
        return false;
      }
    }
  }

  sleep(1.0);

  timer_main_->start();

  sleep(5.0);

  if (uh->getHeightAgl() < 1.5) {
    RCLCPP_ERROR(node_->get_logger(), "the UAV is not as high agl as it should be");
    return false;
  }

  {
    auto [success, message] = uh->takeoff();

    if (success) {
      RCLCPP_ERROR(node_->get_logger(), "takeoff initiated, this should not be possible");
      return false;
    }
  }

  sleep(1.0);

  if (uh->isOutputEnabled()) {

    RCLCPP_ERROR(node_->get_logger(), "control output is still enabled!");
    return false;

  } else {
    return true;
  }
}

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  bool test_result = true;

  Tester tester;

  test_result &= tester.test();

  tester.sleep(2.0);

  std::cout << "Test: reporting test results" << std::endl;

  tester.reportTestResult(test_result);

  tester.join();
}
