#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <algorithm>
#include <iostream>

#include <mrs_uav_testing/test_generic.h>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester() : mrs_uav_testing::TestGeneric() {
  }

  bool test(void);

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh_;

private:
  bool automaticStartRunning(void);
};

bool Tester::automaticStartRunning(void) {
  const auto names = node_->get_node_names();
  return std::find(names.begin(), names.end(), "/uav1/automatic_start") != names.end();
}

bool Tester::test(void) {

  const std::string uav_name = "uav1";

  {
    auto [uhopt, message] = getUAVHandler(uav_name);

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "Failed obtain handler for '%s': '%s'", uav_name.c_str(), message.c_str());
      return false;
    }

    uh_ = uhopt.value();
  }

  while (!uh_->mrsSystemReady()) {

    if (!rclcpp::ok()) {
      return false;
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "waiting for the MRS UAV System");
    sleep(0.01);
  }

  // automatic start must not exist yet, otherwise this test does not test anything
  if (automaticStartRunning()) {
    RCLCPP_ERROR(node_->get_logger(), "automatic start is already running before arming, invalid test setup");
    return false;
  }

  // | ---------------- arm, as a pilot would do ---------------- |

  {
    auto [success, message] = uh_->arming(true);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "arming failed with message: '%s'", message.c_str());
      return false;
    }
  }

  sleep(1.0);

  if (!uh_->isArmed() || uh_->isOutputEnabled()) {
    RCLCPP_ERROR(node_->get_logger(), "expected armed with control output disabled before starting automatic start");
    return false;
  }

  // | ------------ only now bring up automatic start ----------- |

  // test.py watches stdout for this line and launches automatic start in response
  std::cout << "START_AUTOMATIC_START" << std::endl;

  const rclcpp::Time wait_start = clock_->now();

  while (!automaticStartRunning()) {

    if (!rclcpp::ok() || (clock_->now() - wait_start).seconds() > 30.0) {
      RCLCPP_ERROR(node_->get_logger(), "automatic start did not come up");
      return false;
    }

    sleep(0.1);
  }

  // | --- preflight fails here, so output can never be enabled: it must not disarm -- |

  // well past arm_to_output_timeout
  const rclcpp::Time watch_start = clock_->now();

  while ((clock_->now() - watch_start).seconds() < 10.0) {

    if (!rclcpp::ok()) {
      return false;
    }

    if (!uh_->isArmed()) {
      RCLCPP_ERROR(node_->get_logger(), "automatic start disarmed a UAV it did not see being armed");
      return false;
    }

    if (uh_->isOutputEnabled()) {
      RCLCPP_ERROR(node_->get_logger(), "control output got enabled although preflight fails -- invalid test setup");
      return false;
    }

    sleep(0.1);
  }

  return true;
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
