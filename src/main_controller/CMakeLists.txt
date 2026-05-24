cmake_minimum_required(VERSION 3.0.2)
project(main_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  ControllerData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES asr_its
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf visualization_msgs message_runtime
)

###########
## Build ##
###########

set(EIGEN_INCLUDE_DIR "/usr/include/eigen3")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
)

## ---------------------------------------------------------
## 1. Declare Libraries
## ---------------------------------------------------------

add_library(rs232 src/asr_its/rs232.c)

add_library(robot src/asr_its/control_layout.cpp)
# FIX: Tell 'robot' library to wait for message generation
add_dependencies(robot ${PROJECT_NAME}_generate_messages_cpp)

add_library(robot_comhardware src/asr_its/robot_comhardware.cpp)
# FIX: Tell 'robot_comhardware' library to wait for message generation
add_dependencies(robot_comhardware ${PROJECT_NAME}_generate_messages_cpp)

## ---------------------------------------------------------
## 2. Declare Executables
## ---------------------------------------------------------

add_executable(main_node src/main.cpp)
add_executable(comhardware_node src/comhardware.cpp)
add_executable(tf_broadcaster_node src/broadcaster.cpp)

# NOTE: You previously had 'robot_node' compiling 'robot_main.cpp'
add_executable(robot_node src/robot_main.cpp)

# NOTE: You previously had 'robot_comhardware_node' compiling 'robot_comhardware.cpp'
# BUT you also made a library called 'robot_comhardware' above with the same source.
# This is fine, but usually you link the library to the node rather than compiling source twice.
# I kept your original structure here to ensure it works as you intended.
add_executable(robot_comhardware_node src/robot_comhardware.cpp)
# FIX: Add dependency here too just to be safe
add_dependencies(robot_comhardware_node ${PROJECT_NAME}_generate_messages_cpp)


## ---------------------------------------------------------
## 3. Link Libraries
## ---------------------------------------------------------

target_link_libraries(main_node ${catkin_LIBRARIES})
target_link_libraries(comhardware_node rs232 ${catkin_LIBRARIES})
target_link_libraries(tf_broadcaster_node ${catkin_LIBRARIES})

# Link 'robot_node' to the 'robot' library we defined above
target_link_libraries(robot_node robot ${catkin_LIBRARIES} ${EIGEN_INCLUDE_DIR})

# Link 'robot_comhardware_node' to the 'robot_comhardware' library and 'rs232'
target_link_libraries(robot_comhardware_node robot_comhardware rs232 ${catkin_LIBRARIES})

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/GetData.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
