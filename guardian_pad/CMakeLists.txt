
cmake_minimum_required(VERSION 2.8.3)
project(guardian_pad)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs robotnik_msgs message_generation)

add_service_files(DIRECTORY srv 
  FILES
  enable_disable.srv
)

generate_messages(
  DEPENDENCIES
  
)

#generate_messages()

catkin_package(
CATKIN_DEPENDS
    roscpp
    diagnostic_updater
    robotnik_msgs
)

#include_directories(include ${PROJECT_SOURCE_DIR}/include/guardian_pad ${PROJECT_SOURCE_DIR}/srv_gen/cpp/include/guardian_pad ${catkin_INCLUDE_DIRS})

include_directories(include ${PROJECT_SOURCE_DIR}/include/guardian_pad ${catkin_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
add_executable(guardian_pad src/guardian_pad.cpp)
target_link_libraries(guardian_pad ${catkin_LIBRARIES})
add_dependencies(guardian_pad ${catkin_EXPORTED_TARGETS})

#add_executable(guardian_pad src/guardian_pad.cpp ../guardian_node/src/Component.cc ../guardian_node/src/guardian_controller.cc ../guardian_node/src/RoboteqDevice.cpp)
#target_link_libraries(guardian_pad ${catkin_LIBRARIES})
#add_dependencies(guardian_pad ${catkin_EXPORTED_TARGETS})


