#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester() : mrs_uav_testing::TestGeneric() {
  }

  bool test(void);
};

bool Tester::test(void) {

  const std::string uav_name = "uav1";

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh;

  {
    auto [uhopt, message] = getUAVHandler(uav_name);

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed obtain handler for '%s': '%s'", uav_name.c_str(), message.c_str());
      return false;
    }

    uh = uhopt.value();
  }

  // | ---------------- wait for ready to takeoff --------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return false;
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "waiting for the MRS UAV System");

    if (uh->mrsSystemReady()) {
      RCLCPP_INFO(node_->get_logger(), "MRS UAV System is ready");
      break;
    }

    sleep(0.01);
  }

  // | ---------------------- arm the drone --------------------- |

  RCLCPP_INFO(node_->get_logger(), "[%s]: arming the drone", name_.c_str());

  {
    auto [success, message] = uh->arming(true);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "midair activation failed with message: '%s'", message.c_str());
      return false;
    }
  }

  // | ---------------------- wait a second --------------------- |

  sleep(2.0);

  // | --------------------- check if armed --------------------- |

  if (!uh->sh_hw_api_status_.getMsg()->armed) {
    return false;
  }

  // | ------------------- switch to offboard ------------------- |

  {
    auto [success, message] = uh->offboard();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "offboard activation failed with message: '%s'", message.c_str());
      return false;
    }
  }

  // | -------------------------- wait -------------------------- |

  sleep(2.0);

  // | ------------------ check if in offboard ------------------ |

  if (!uh->sh_hw_api_status_.getMsg()->offboard) {
    return false;
  }

  sleep(2.0);

  // | ------------------------- takeoff ------------------------ |

  {
    auto [success, message] = uh->takeoffService();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "takeoff failed with message: '%s'", message.c_str());
      return false;
    }
  }

  // | --------------- wait for takeoff to finish --------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return false;
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "waiting for the takeoff to finish");

    if (uh->sh_control_manager_diag_.getMsg()->flying_normally) {

      return true;
    }

    sleep(0.01);
  }

  return false;
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
