cmake_minimum_required(VERSION 2.8.3)
project(jaco_teach CXX C)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  kinova_driver
  kinova_msgs
  std_msgs
  roscpp
)

catkin_package(
)

find_package(PkgConfig)

pkg_check_modules(GTKMM gtkmm-3.0)

link_directories(
    ${GTKMM_LIBRARY_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTKMM_INCLUDE_DIRS}
)

add_library(jaco_teach include/jaco_teach.cpp)

add_executable(bringup src/main.cpp)

target_link_libraries(bringup
  jaco_teach
  ${catkin_LIBRARIES}
  ${GTKMM_LIBRARIES}
)
