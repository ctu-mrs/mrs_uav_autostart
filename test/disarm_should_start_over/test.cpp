#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester() : mrs_uav_testing::TestGeneric() {
  }

  bool test(void);

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh_;

private:
  bool waitFor(const std::function<bool()> &pred, const double timeout);
};

bool Tester::waitFor(const std::function<bool()> &pred, const double timeout) {

  const rclcpp::Time start = clock_->now();

  while (rclcpp::ok() && (clock_->now() - start).seconds() < timeout) {

    if (pred()) {
      return true;
    }

    sleep(0.1);
  }

  return false;
}

bool Tester::test(void) {

  const std::string uav_name = "uav1";

  {
    auto [uhopt, message] = getUAVHandler(uav_name);

    if (!uhopt) {
      RCLCPP_ERROR(node_->get_logger(), "obtain handler for '%s': '%s'", uav_name.c_str(), message.c_str());
      return false;
    }

    uh_ = uhopt.value();
  }

  if (!waitFor([&] { return uh_->mrsSystemReady(); }, 60.0)) {
    RCLCPP_ERROR(node_->get_logger(), "the MRS UAV System is not ready");
    return false;
  }

  // | ------- arm, let automatic start enable the output ------- |

  {
    auto [success, message] = uh_->arming(true);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "arming failed with message: '%s'", message.c_str());
      return false;
    }
  }

  if (!waitFor([&] { return uh_->isOutputEnabled(); }, 10.0)) {
    RCLCPP_ERROR(node_->get_logger(), "automatic start did not enable the output after arming");
    return false;
  }

  // | ------------ disarm before going to offboard ------------- |

  {
    auto [success, message] = uh_->arming(false);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "disarming failed with message: '%s'", message.c_str());
      return false;
    }
  }

  if (!waitFor([&] { return !uh_->isArmed(); }, 5.0)) {
    RCLCPP_ERROR(node_->get_logger(), "the UAV did not disarm");
    return false;
  }

  // longer than arm_to_output_timeout: automatic start must keep waiting, not finish
  sleep(3.0);

  // | --------- arm again: a fresh start, then take off -------- |

  {
    auto [success, message] = uh_->takeoff();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "takeoff after disarm -> arm failed with message: '%s'", message.c_str());
      return false;
    }
  }

  this->sleep(5.0);

  if (!uh_->isFlyingNormally()) {
    RCLCPP_ERROR(node_->get_logger(), "not flying normally after the takeoff");
    return false;
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
