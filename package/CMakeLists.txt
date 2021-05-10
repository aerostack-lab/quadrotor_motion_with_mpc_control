cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_motion_with_mpc_control)

### Use version 2011 of C++ (c++11). By default ROS uses c++98
#see: http://stackoverflow.com/questions/10851247/how-to-activate-c-11-in-cmake
#see: http://stackoverflow.com/questions/10984442/how-to-detect-c11-support-of-a-compiler-with-cmake
add_definitions(-std=c++11)


# Directories definition
set(QUADROTOR_MOTION_WITH_MPC_CONTROL_SOURCE_DIR
  src/source
)

set(QUADROTOR_MOTION_WITH_MPC_CONTROL_INCLUDE_DIR
  src/include
)

set(QUADROTOR_MOTION_WITH_MPC_CONTROL_HEADER_FILES
        ${QUADROTOR_MOTION_WITH_MPC_CONTROL_INCLUDE_DIR}/behavior_hover_with_mpc_control.h 
        ${QUADROTOR_MOTION_WITH_MPC_CONTROL_INCLUDE_DIR}/behavior_follow_path.h 
        ${QUADROTOR_MOTION_WITH_MPC_CONTROL_INCLUDE_DIR}/behavior_rotate_with_mpc_control.h
        ${QUADROTOR_MOTION_WITH_MPC_CONTROL_INCLUDE_DIR}/behavior_quadrotor_mpc_motion_control.h
)

set(QUADROTOR_MOTION_WITH_MPC_CONTROL_SOURCE_FILES
        ${QUADROTOR_MOTION_WITH_MPC_CONTROL_SOURCE_DIR}/behavior_hover_with_mpc_control.cpp 
        ${QUADROTOR_MOTION_WITH_MPC_CONTROL_SOURCE_DIR}/behavior_follow_path.cpp 
        ${QUADROTOR_MOTION_WITH_MPC_CONTROL_SOURCE_DIR}/behavior_rotate_with_mpc_control.cpp
        ${QUADROTOR_MOTION_WITH_MPC_CONTROL_SOURCE_DIR}/behavior_quadrotor_mpc_motion_control.cpp
)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  std_msgs
  behavior_execution_controller
  behavior_execution_manager_msgs
  geometry_msgs
  aerostack_msgs
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
  INCLUDE_DIRS ${QUADROTOR_MOTION_WITH_MPC_CONTROL_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp 
  nodelet
  std_msgs
  behavior_execution_controller
  behavior_execution_manager_msgs
  geometry_msgs
  trajectory_msgs
  aerostack_msgs
  DEPENDS yaml-cpp
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
add_library(quadrotor_motion_with_mpc_control
  src/source/behavior_hover_with_mpc_control.cpp 
  src/source/behavior_follow_path.cpp 
  src/source/behavior_rotate_with_mpc_control.cpp
  src/source/behavior_quadrotor_mpc_motion_control.cpp   
)
target_link_libraries(quadrotor_motion_with_mpc_control ${catkin_LIBRARIES})
target_link_libraries(quadrotor_motion_with_mpc_control yaml-cpp)
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(quadrotor_motion_with_mpc_control ${catkin_EXPORTED_LIBRARIES})
endif()
