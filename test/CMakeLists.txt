find_package(rostest REQUIRED)

# takeoff test

add_rostest_gtest(MrsUavAutostart_basic
  basic/basic.test
  basic/test.cpp
  )

target_link_libraries(
  MrsUavAutostart_basic
  ${catkin_LIBRARIES}
  )

add_dependencies(
  MrsUavAutostart_basic
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
