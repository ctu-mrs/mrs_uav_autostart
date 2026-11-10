#include <rclcpp/rclcpp.hpp>

#include <mrs_msgs/msg/errorgraph_element.hpp>
#include <mrs_msgs/msg/errorgraph_error.hpp>

#include <mrs_uav_testing/test_generic.h>

#include <mrs_mock.hpp>

#include <atomic>

class Tester : public mrs_uav_testing::TestGeneric {

public:
  bool test(void);
};

bool Tester::test(void) {

  MrsMock mrs(node_);
  mrs.publish_general_robot_info = false; // held back until the waiting error was seen

  std::atomic<bool> released{false};
  std::atomic<bool> saw_waiting_error{false};
  std::atomic<int>  clean_after_release{0};

  // the shared errors topic carries every node's errors, only automatic start's are checked
  auto sub = node_->create_subscription<mrs_msgs::msg::ErrorgraphElement>("/uav1/errors", 100, [&](const mrs_msgs::msg::ErrorgraphElement::ConstSharedPtr msg) {
    if (msg->source_node.node != "AutomaticStart" || msg->source_node.component != "main") {
      return;
    }

    bool waiting = false;
    for (const auto &error : msg->errors) {
      if (error.type == mrs_msgs::msg::ErrorgraphError::TYPE_WAITING_FOR_NODE) {
        waiting = true;
      }
    }

    if (!released) {
      saw_waiting_error = saw_waiting_error || waiting;
    } else {
      clean_after_release = waiting ? 0 : clean_after_release + 1;
    }
  });

  if (!mrs.waitFor([&] { return saw_waiting_error.load(); }, 30.0)) {
    return mrs.fail("automatic_start never reported waiting for DiagnosticsManager");
  }

  released                       = true;
  mrs.publish_general_robot_info = true;

  if (!mrs.waitFor([&] { return clean_after_release >= 3; }, 15.0)) {
    return mrs.fail("automatic_start's waiting error did not clear once DiagnosticsManager's data came");
  }

  return true;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  Tester     tester;
  const bool result = tester.test();
  tester.sleep(1.0);
  tester.reportTestResult(result);
  tester.join();
}
