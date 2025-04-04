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

  ph_velocity_ = mrs_lib::PublisherHandler<mrs_msgs::HwApiVelocityHdgCmd>(nh_, "/multirotor_simulator/uav1/velocity_hdg_cmd");

  timer_main_ = nh_.createTimer(ros::Rate(100.0), &Tester::timerMain, this, false, false);
}

void Tester::timerMain([[maybe_unused]] const ros::TimerEvent& event) {

  mrs_msgs::HwApiVelocityHdgCmd msg;

  msg.velocity.x = 1.0;
  msg.velocity.z = 1.5;

  ph_velocity_.publish(msg);
}

bool Tester::test() {

  std::shared_ptr<mrs_uav_testing::UAVHandler> uh;

  {
    auto [uhopt, message] = getUAVHandler(_uav_name_);

    if (!uhopt) {
      ROS_ERROR("[%s]: Failed obtain handler for '%s': '%s'", ros::this_node::getName().c_str(), _uav_name_.c_str(), message.c_str());
      return false;
    }

    uh = uhopt.value();
  }

  {
    while (ros::ok()) {
      if (uh->mrsSystemReady()) {
        break;
      }
    }
  }

  sleep(1.0);

  {
    std_srvs::SetBool srv;

    srv.request.data = true;

    if (!uh->sch_arming_.call(srv)) {
      ROS_ERROR("[%s]: Failed to arm the UAV '%s'", ros::this_node::getName().c_str(), srv.response.message.c_str());
      return false;
    }
  }

  sleep(1.0);

  timer_main_.start();

  sleep(5.0);

  if (uh->getHeightAgl() < 1.5) {
    ROS_ERROR("[%s]: the UAV is not as high agl as it should be", ros::this_node::getName().c_str());
    return false;
  }

  {
    auto [success, message] = uh->takeoff();

    if (success) {
      ROS_ERROR("[%s]: takeoff initiated, this should not be possible", ros::this_node::getName().c_str());
      return false;
    }
  }

  sleep(1.0);

  if (uh->isOutputEnabled()) {

    ROS_ERROR("[%s]: control output is still enabled!", ros::this_node::getName().c_str());
    return false;

  } else {
    return true;
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
