cmake_minimum_required(VERSION 2.8.3)
project(crane_x7_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  rospy
  actionlib_msgs
  std_msgs
  dynamic_reconfigure
  message_generation
)

generate_dynamic_reconfigure_options(
  cfg/ServoParameter.cfg
)

add_message_files(
  FILES
  ControlInputPolicy.msg
  JointPositionCommand.msg
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
  # LIBRARIES ${PROJECT_NAME}
  # CATKIN_DEPENDS 
  # message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
