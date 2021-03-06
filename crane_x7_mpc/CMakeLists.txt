cmake_minimum_required(VERSION 3.0.2)
project(crane_x7_mpc)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  nodelet
  std_msgs
  message_generation
)

add_service_files(
  FILES
  SetGoalConfiguration.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp 
  message_runtime
  nodelet
)

# Build IDOCP
set(IDOCP_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/IDOCP/include/IDOCP)
set(IDOCP_SRC_DIR ${PROJECT_SOURCE_DIR}/IDOCP/src)

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED )
find_package(PkgConfig)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)
add_definitions(
  -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
  -DPINOCCHIO_WITH_URDFDOM
)
link_directories(${PINOCCHIO_LIBDIR})

find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
  set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_library(
  robot
  SHARED 
  ${IDOCP_SRC_DIR}/robot/robot.cpp
  ${IDOCP_SRC_DIR}/robot/passive_joints.cpp
  ${IDOCP_SRC_DIR}/robot/point_contact.cpp
)
target_include_directories(
  robot
  PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  ${IDOCP_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDEDIR}
)
target_link_libraries(
  robot
  PRIVATE
  pthread
  ${Boost_LIBRARIES}
  urdfdom_model
  ${PINOCCHIO_LIBRARIES}
)

add_library(
  constraints_pdipm
  SHARED 
  ${IDOCP_SRC_DIR}/constraints/pdipm/joint_space_constraints_pdipm.cpp
  ${IDOCP_SRC_DIR}/constraints/pdipm/joint_position_upper_limits_pdipm.cpp
  ${IDOCP_SRC_DIR}/constraints/pdipm/joint_position_lower_limits_pdipm.cpp
  ${IDOCP_SRC_DIR}/constraints/pdipm/joint_velocity_upper_limits_pdipm.cpp
  ${IDOCP_SRC_DIR}/constraints/pdipm/joint_velocity_lower_limits_pdipm.cpp
  ${IDOCP_SRC_DIR}/constraints/pdipm/joint_torque_upper_limits_pdipm.cpp
  ${IDOCP_SRC_DIR}/constraints/pdipm/joint_torque_lower_limits_pdipm.cpp
  ${IDOCP_SRC_DIR}/constraints/pdipm/pdipm_func.cpp
)
target_include_directories(
  constraints_pdipm
  PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  ${IDOCP_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDEDIR}
)
target_link_libraries(
  constraints_pdipm
  PRIVATE
  robot
)

add_library(
  idocp
  SHARED 
  ${IDOCP_SRC_DIR}/ocp/ocp.cpp
  ${IDOCP_SRC_DIR}/ocp/split_ocp.cpp
  ${IDOCP_SRC_DIR}/ocp/line_search_filter.cpp
  ${IDOCP_SRC_DIR}/cost/joint_space_cost.cpp
  ${IDOCP_SRC_DIR}/manipulator/cost_function.cpp
  ${IDOCP_SRC_DIR}/manipulator/constraints.cpp
)
target_include_directories(
  idocp
  PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  ${IDOCP_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDEDIR}
)
target_link_libraries(
  idocp
  PRIVATE
  robot
  constraints_pdipm
)

add_library(
  mpc
  SHARED 
  ${IDOCP_SRC_DIR}/ocp/mpc.cpp
)
target_include_directories(
  mpc
  PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  ${IDOCP_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDEDIR}
)
target_link_libraries(
  mpc
  PRIVATE
  idocp
  robot
  constraints_pdipm
)

# Build mpc_nodelet
add_library(
  mpc_nodelet
  src/mpc_nodelet.cpp
)
target_link_libraries(
  mpc_nodelet
  PRIVATE
  mpc
  idocp
  robot
  constraints_pdipm
  ${catkin_LIBRARIES}
)
target_include_directories(
  mpc_nodelet
  PRIVATE
  ${PROJECT_SOURCE_DIR}/include/crane_x7_mpc
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${IDOCP_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDEDIR}
)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(mpc_nodelet ${catkin_EXPORTED_LIBRARIES})
endif()