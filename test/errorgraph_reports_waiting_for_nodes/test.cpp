#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>
#include <mrs_msgs/msg/errorgraph_element.hpp>
#include <mrs_msgs/msg/errorgraph_error.hpp>

#include <mutex>
#include <vector>
#include <set>
#include <string>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester();

  bool test(void);

private:
  // Subscription for errorgraph errors - created early to catch startup errors
  rclcpp::Subscription<mrs_msgs::msg::ErrorgraphElement>::SharedPtr sub_errors_;
  std::mutex                                                        errors_mtx_;
  std::vector<mrs_msgs::msg::ErrorgraphElement>                     received_errors_;

  void errorsCallback(const mrs_msgs::msg::ErrorgraphElement::SharedPtr msg);
};

Tester::Tester() : mrs_uav_testing::TestGeneric() {

  // Subscribe to the errorgraph errors topic immediately, before blocking on getUAVHandler.
  // This ensures we capture errors published during the startup window.
  sub_errors_ = node_->create_subscription<mrs_msgs::msg::ErrorgraphElement>("/uav1/automatic_start/errors", 100,
                                                                             std::bind(&Tester::errorsCallback, this, std::placeholders::_1));
}

void Tester::errorsCallback(const mrs_msgs::msg::ErrorgraphElement::SharedPtr msg) {
  std::scoped_lock lck(errors_mtx_);
  received_errors_.push_back(*msg);
}

bool Tester::test(void) {

  RCLCPP_INFO(node_->get_logger(), "Waiting for errorgraph messages with waiting_for_node errors...");

  // Wait up to 30 seconds for at least one ErrorgraphElement with waiting_for_node errors.
  // The ErrorPublisher publishes at 1Hz. During startup, AutomaticStart adds
  // waiting_for_node errors every timerMain cycle (30Hz). We should catch at least
  // one publish cycle with errors before all dependencies come online.

  const double timeout_s           = 30.0;
  const double poll_rate_s         = 0.1;
  double       elapsed             = 0.0;
  bool         found_waiting_error = false;

  // The set of expected dependency node names
  const std::set<std::string> expected_nodes = {"HwApiManager", "ControlManager", "UavManager", "EstimationManager"};

  std::set<std::string> found_nodes;

  while (rclcpp::ok() && elapsed < timeout_s) {

    {
      std::scoped_lock lck(errors_mtx_);

      for (const auto &element : received_errors_) {

        // Verify source_node identity
        if (element.source_node.node != "AutomaticStart" || element.source_node.component != "main") {
          continue;
        }

        for (const auto &error : element.errors) {
          if (error.type == mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_NODE) {
            found_waiting_error = true;
            found_nodes.insert(error.waited_for_node.node);
          }
        }
      }
    }

    if (found_waiting_error) {
      break;
    }

    sleep(poll_rate_s);
    elapsed += poll_rate_s;
  }

  if (!found_waiting_error) {
    RCLCPP_ERROR(node_->get_logger(), "FAILED: Did not receive any errorgraph messages with waiting_for_node errors within %.1f seconds", timeout_s);
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "SUCCESS: Found waiting_for_node errors. Detected nodes:");
  for (const auto &n : found_nodes) {
    RCLCPP_INFO(node_->get_logger(), "  - %s", n.c_str());
  }

  // Verify that at least one of the expected nodes was reported
  bool any_expected = false;
  for (const auto &n : found_nodes) {
    if (expected_nodes.count(n) > 0) {
      any_expected = true;
    }
  }

  if (!any_expected) {
    RCLCPP_ERROR(node_->get_logger(), "FAILED: Found waiting_for_node errors but none matched expected dependency nodes");
    return false;
  }

  return true;
}

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  bool test_result = true;

  Tester tester;

  test_result &= tester.test();

  // Sleep long enough for the Python test harness subscriber to connect
  // to /test_result before we publish. The errorgraph test completes quickly
  // (~2s), so we need extra time for peer discovery.
  tester.sleep(10.0);

  std::cout << "Test: reporting test results" << std::endl;

  // Publish the result multiple times to ensure the Python harness receives it,
  // since single-shot publishes can be missed with volatile QoS.
  for (int i = 0; i < 5; i++) {
    tester.reportTestResult(test_result);
    tester.sleep(1.0);
  }

  tester.join();
}
