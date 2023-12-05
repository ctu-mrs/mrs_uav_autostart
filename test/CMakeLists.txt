find_package(rostest REQUIRED)
find_package(catkin REQUIRED COMPONENTS mrs_uav_testing)

# will test takeoff in mrs simulator
add_subdirectory(takeoff)

# will test takeoff in gazebo
add_subdirectory(gazebo_takeoff)

# will test if takeoff is not allowed outside of safety area
add_subdirectory(outside_safety_area)

# will test the "handle_takeoff: false" option
add_subdirectory(without_takeoff)

# will test if takeoff is not allowed when UAV is moving
add_subdirectory(moving_drone)

# will test if takeoff is not allowed if topic check fails
add_subdirectory(topic_check)
