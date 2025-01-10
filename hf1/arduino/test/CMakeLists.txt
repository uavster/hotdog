cmake_minimum_required(VERSION 2.8)
project(hf1_arduino_tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Setup testing
enable_testing()

find_package(GTest REQUIRED)
if(DEFINED GTEST_INCLUDE_DIR)
  include_directories(${GTEST_INCLUDE_DIR})
endif()

get_filename_component(PARENT_DIR ../ ABSOLUTE)
include_directories(${PARENT_DIR})
include_directories(${PARENT_DIR}/../common)

set(TEST_SOURCES
  trajectory_test.cpp
  quaternion2_test.cpp
)

# Add test cpp file.
add_executable(runArduinoTests ${ARDUINO_SOURCES} ${TEST_SOURCES})

# Link test executable against all dependency libraries.
if(NOT DEFINED GTEST_INCLUDE_DIR)
  target_link_libraries(runArduinoTests hf1_arduino_test_lib pthread GTest::gtest_main)
else()
  target_link_libraries(runArduinoTests hf1_arduino_test_lib libgtest.a libgtest_main.a pthread)
endif()

add_test(
  NAME runArduinoTests
  COMMAND runArduinoTests
)
set_tests_properties(runArduinoTests PROPERTIES DEPENDS hf1_arduino_tests)
add_custom_target(check_arduino COMMAND ${CMAKE_CTEST_COMMAND}
                  DEPENDS runArduinoTests)