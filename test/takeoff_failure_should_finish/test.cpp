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

  {
    auto [success, message] = uh_->offboard();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "offboard failed with message: '%s'", message.c_str());
      return false;
    }
  }

  // automatic start triggers the takeoff after takeoff_countdown (5 s); UavManager rejects it (WrongTracker)
  sleep(10.0);

  if (uh_->isFlyingNormally()) {
    RCLCPP_ERROR(node_->get_logger(), "the UAV took off, but the takeoff should have been rejected");
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
