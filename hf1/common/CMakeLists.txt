project(hf1_p2p_link_common)
cmake_minimum_required(VERSION 2.8)
add_library(hf1_p2p_link_common network.cpp p2p_packet_stream.cpp logger_interface.cpp)
target_include_directories(hf1_p2p_link_common PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
