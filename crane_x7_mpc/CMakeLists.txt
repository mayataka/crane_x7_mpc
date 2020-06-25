cmake_minimum_required(VERSION 2.8.3)
project(crane_x7_mpc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  controller_interface
  hardware_interface
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp nodelet
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(mpc_nodelet
  src/mpc_nodelet.cpp
)
target_link_libraries(mpc_nodelet ${catkin_LIBRARIES})
target_include_directories(
  mpc_nodelet
  PRIVATE
  ${PROJECT_SOURCE_DIR}/include/crane_x7_mpc
)
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(mpc_nodelet ${catkin_EXPORTED_LIBRARIES})
endif()
