cmake_minimum_required(VERSION 3.0.2)
project(crane_x7_mpc)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(robotoc REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  nodelet
  std_msgs
  crane_x7_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp 
  nodelet
)

add_library(
  mpc_nodelet
  src/mpc_nodelet.cpp
)
target_link_libraries(
  mpc_nodelet
  PUBLIC
  robotoc::robotoc
  ${catkin_LIBRARIES}
)
target_include_directories(
  mpc_nodelet
  PUBLIC
  include/crane_x7_mpc
  ${ROBOTOC_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(mpc_nodelet ${catkin_EXPORTED_LIBRARIES})
endif()