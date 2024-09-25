cmake_minimum_required(VERSION 3.8)

project(WiSUNDebugger VERSION 1.0 LANGUAGES CXX)

add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(wisun_tx simple_udp.cpp wisun_tx.cpp)

target_include_directories(wisun_tx PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>)

add_executable(wisun_rx simple_udp.cpp wisun_rx.cpp)

target_include_directories(wisun_rx PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>)