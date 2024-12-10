project(hf1_p2p_link_arduino)
cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_subdirectory(test)

get_filename_component(PARENT_DIR ../ ABSOLUTE)
include_directories(${PARENT_DIR}/common)

set(TEST_SOURCES
  controller.cpp
  head_controller.cpp
  logger.cpp
  p2p_action_server.cpp
  periodic_runnable.cpp
  pid.cpp
  point.cpp
  set_base_velocity_action_handler.cpp
  set_head_pose_action_handler.cpp
  wheel_controller.cpp
)

add_library(hf1_arduino_test_lib ${TEST_SOURCES})
target_link_libraries(hf1_arduino_test_lib hf1_p2p_link_common)
