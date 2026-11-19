ament_add_gtest(test_state_checks test.cpp)

target_include_directories(test_state_checks PRIVATE ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(test_state_checks
  ${mrs_msgs_TARGETS}
)
