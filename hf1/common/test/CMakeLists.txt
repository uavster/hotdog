cmake_minimum_required(VERSION 2.8)
project(hf1_p2p_link_common_tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Setup testing
enable_testing()
find_package(GTest REQUIRED)

get_filename_component(PARENT_DIR ../ ABSOLUTE)
include_directories(${PARENT_DIR})

# Add test cpp file.
add_executable(runUnitTests
    ring_buffer_test.cpp
)

# Link test executable against all dependency libraries.
target_link_libraries(runUnitTests hf1_p2p_link_common pthread GTest::gtest_main)

add_test(
    NAME runUnitTests
    COMMAND runUnitTests
)
set_tests_properties(runUnitTests PROPERTIES DEPENDS hf1_p2p_link_common_tests)
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND}
                  DEPENDS runUnitTests)