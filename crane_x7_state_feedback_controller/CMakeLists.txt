cmake_minimum_required(VERSION 3.0.2)
project(crane_x7_state_feedback_controller)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
  hardware_interface
  crane_x7_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp 
  controller_interface 
)

find_package(Eigen3 REQUIRED)

add_library(
  state_feedback_controller
  src/state_feedback_controller.cpp
)

target_include_directories(
  state_feedback_controller
  PRIVATE
  include/crane_x7_state_feedback_controller
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(state_feedback_controller ${catkin_EXPORTED_LIBRARIES})
endif()