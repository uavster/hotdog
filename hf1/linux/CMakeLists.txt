project(hf1_p2p_link_linux)
cmake_minimum_required(VERSION 2.8)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../common ${OpenCV_INCLUDE_DIRS})
add_library(hf1_p2p_link_linux p2p_byte_stream_linux.cpp guid_factory.cpp time_sync_client.cpp uart.cpp)
target_include_directories(hf1_p2p_link_linux PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
