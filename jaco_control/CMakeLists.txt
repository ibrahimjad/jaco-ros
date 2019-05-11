cmake_minimum_required(VERSION 2.8.3)
project(jaco_control CXX C)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  kinova_driver
  kinova_msgs
  std_msgs
  roscpp
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(jaco_control include/jaco_control.cpp)

add_executable(bringup src/main.cpp)

target_link_libraries(bringup
  jaco_control
  ${catkin_LIBRARIES}
)
