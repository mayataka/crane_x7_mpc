cmake_minimum_required(VERSION 3.0.2)
project(crane_x7_mpc)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(robotoc REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  crane_x7_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp 
)

add_executable(
  ${PROJECT_NAME}
  src/mpc.cpp
  src/mpc_node.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
  robotoc::robotoc
  ${catkin_LIBRARIES}
)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  include/crane_x7_mpc
  ${ROBOTOC_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

foreach(dir config launch)
  install(DIRECTORY ${dir}/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach(dir)