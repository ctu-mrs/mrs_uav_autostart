#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mrs_uav_testing/test_generic.h>
#include <mrs_msgs/msg/errorgraph_element.hpp>
#include <mrs_msgs/msg/errorgraph_error.hpp>

#include <mutex>
#include <vector>
#include <string>
#include <optional>

using namespace std::chrono_literals;

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester();

  bool test(void);

private:
  rclcpp::Subscription<mrs_msgs::msg::ErrorgraphElement>::SharedPtr sub_errors_;
  std::mutex                                                        errors_mtx_;
  std::optional<mrs_msgs::msg::ErrorgraphElement>                   last_msg_;

  void errorsCallback(const mrs_msgs::msg::ErrorgraphElement::SharedPtr msg);
};

Tester::Tester() : mrs_uav_testing::TestGeneric() {

  // Subscribe immediately so we capture the full lifecycle:
  // waiting_for_node errors during startup -> empty errors after startup
  sub_errors_ = node_->create_subscription<mrs_msgs::msg::ErrorgraphElement>("/uav1/automatic_start/errors", 100,
                                                                             std::bind(&Tester::errorsCallback, this, std::placeholders::_1));
}

void Tester::errorsCallback(const mrs_msgs::msg::ErrorgraphElement::SharedPtr msg) {
  std::scoped_lock lck(errors_mtx_);
  last_msg_ = *msg;
}

bool Tester::test(void) {

  RCLCPP_INFO(node_->get_logger(), "Waiting for errorgraph errors to clear after startup...");

  // We expect this lifecycle:
  //   1. AutomaticStart publishes waiting_for_node errors during startup
  //   2. Once all dependencies are ready, it stops adding errors
  //   3. ErrorPublisher clears after each publish -> subsequent messages have empty errors
  //
  // We poll for up to 90 seconds for N consecutive messages with no waiting_for_node errors.

  const double timeout_s                  = 90.0;
  const double poll_rate_s                = 0.2;
  const int    required_consecutive_clean = 3;

  double elapsed               = 0.0;
  int    consecutive_clean     = 0;
  bool   saw_any_waiting_error = false;

  while (rclcpp::ok() && elapsed < timeout_s) {

    sleep(poll_rate_s);
    elapsed += poll_rate_s;

    std::scoped_lock lck(errors_mtx_);

    if (!last_msg_.has_value()) {
      continue;
    }

    const auto &element = last_msg_.value();

    if (element.source_node.node != "AutomaticStart" || element.source_node.component != "main") {
      continue;
    }

    bool has_waiting_for_node = false;
    for (const auto &error : element.errors) {
      if (error.type == mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_NODE) {
        has_waiting_for_node  = true;
        saw_any_waiting_error = true;
        break;
      }
    }

    if (!has_waiting_for_node) {
      consecutive_clean++;
      RCLCPP_INFO(node_->get_logger(), "Clean message %d/%d (errors: %zu)", consecutive_clean, required_consecutive_clean, element.errors.size());
    } else {
      consecutive_clean = 0;
    }

    if (consecutive_clean >= required_consecutive_clean) {
      break;
    }
  }

  if (consecutive_clean < required_consecutive_clean) {
    RCLCPP_ERROR(node_->get_logger(),
                 "FAILED: errorgraph errors did not clear within %.1f seconds "
                 "(saw_waiting_errors=%s, consecutive_clean=%d/%d)",
                 timeout_s, saw_any_waiting_error ? "true" : "false", consecutive_clean, required_consecutive_clean);
    return false;
  }

  if (!saw_any_waiting_error) {
    RCLCPP_WARN(node_->get_logger(), "WARNING: errors cleared but no waiting_for_node errors were observed "
                                     "during startup (startup window may have been missed)");
  }

  RCLCPP_INFO(node_->get_logger(), "SUCCESS: errorgraph errors cleared after startup (%d consecutive clean messages).", consecutive_clean);
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
