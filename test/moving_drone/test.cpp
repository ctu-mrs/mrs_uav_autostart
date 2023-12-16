#include <gtest/gtest.h>

#include <mrs_uav_testing/test_generic.h>

#include <mrs_msgs/HwApiVelocityHdgCmd.h>

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester();

  bool test();

  mrs_lib::PublisherHandler<mrs_msgs::HwApiVelocityHdgCmd> ph_velocity_;

  ros::Timer timer_main_;
  void       timerMain(const ros::TimerEvent& event);
};

Tester::Tester() : mrs_uav_testing::TestGeneric() {

  ph_velocity_ = mrs_lib::PublisherHandler<mrs_msgs::HwApiVelocityHdgCmd>(nh_, "/multirotor_simulator/" + _uav_name_ + "/velocity_hdg_cmd");

  timer_main_ = nh_.createTimer(ros::Rate(100.0), &Tester::timerMain, this, false, true);
}

void Tester::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  mrs_msgs::HwApiVelocityHdgCmd msg;

  msg.velocity.x = 0.5;
  msg.velocity.z = 0.1;

  ph_velocity_.publish(msg);
}

bool Tester::test() {

  auto [success, message] = takeoff();

  if (!success) {
    return true;
  } else {
    ROS_ERROR("[%s]: takeoff initiated, this should not be possible", ros::this_node::getName().c_str());
    return false;
  }
}


TEST(TESTSuite, test) {

  Tester tester;

  bool result = tester.test();

  if (result) {
    GTEST_SUCCEED();
  } else {
    GTEST_FAIL();
  }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  ros::init(argc, argv, "test");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
