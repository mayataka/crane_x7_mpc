cmake_minimum_required(VERSION 3.0.2)
project(crane_x7_state_feedback_controller)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
  hardware_interface
  std_msgs
  sensor_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  controller_interface 
  roscpp 
  sensor_msgs 
  std_msgs
)

find_package(Eigen3 REQUIRED)

add_library(
  state_feedback_controller
  ${PROJECT_SOURCE_DIR}/src/state_feedback_controller.cpp
)
target_include_directories(
  state_feedback_controller
  PRIVATE
  ${PROJECT_SOURCE_DIR}/include/crane_x7_state_feedback_controller
)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(state_feedback_controller ${catkin_EXPORTED_LIBRARIES})
endif()