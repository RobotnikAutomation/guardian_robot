cmake_minimum_required(VERSION 2.8.3)
project(guardian_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    tf
    nav_msgs 
    diagnostic_updater
)

add_message_files(
  FILES
  guardian_state.msg
)

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv 
  FILES
  config_pid.srv
  set_odometry.srv
  set_pid.srv
)

generate_messages(
  DEPENDENCIES
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
#  LIBRARIES summit_xl_control
    CATKIN_DEPENDS
    roscpp
    tf
    nav_msgs 
    diagnostic_updater
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(guardian_node_nxt src/guardian_node.cpp src/Component.cc src/guardian_controller.cc src/RoboteqDevice.cpp)
add_executable(guardian_node_ax src/guardian_node_ax.cpp src/Component.cc src/ax3500.cc src/SerialDevice.cc)

add_dependencies(guardian_node_nxt guardian_node_generate_messages_cpp)
add_dependencies(guardian_node_ax guardian_node_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(guardian_node_nxt
  ${catkin_LIBRARIES}
)

target_link_libraries(guardian_node_ax
  ${catkin_LIBRARIES}
)
