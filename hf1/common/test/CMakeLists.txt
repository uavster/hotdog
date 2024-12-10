cmake_minimum_required(VERSION 2.8)
project(hf1_common_tests)

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

# Add test cpp file.
add_executable(runCommonTests
    ring_buffer_test.cpp
)

# Link test executable against all dependency libraries.
if(NOT DEFINED GTEST_INCLUDE_DIR)
  target_link_libraries(runCommonTests hf1_p2p_link_common pthread GTest::gtest_main)
else()
  target_link_libraries(runCommonTests hf1_p2p_link_common libgtest.a libgtest_main.a pthread)
endif()

add_test(
    NAME runCommonTests
    COMMAND runCommonTests
)
set_tests_properties(runCommonTests PROPERTIES DEPENDS hf1_common_tests)
add_custom_target(check_common COMMAND ${CMAKE_CTEST_COMMAND}
                  DEPENDS runCommonTests)