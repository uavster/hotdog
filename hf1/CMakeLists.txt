project(hf1_p2p)
cmake_minimum_required(VERSION 2.8)
add_subdirectory(arduino)
add_subdirectory(common)
add_subdirectory(linux)

add_custom_target(check DEPENDS check_common check_arduino)