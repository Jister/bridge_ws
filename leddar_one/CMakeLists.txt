cmake_minimum_required(VERSION 2.8.3)
project(leddar_one)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Leddar.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES leddar_one
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


SET(CMAKE_C_FLAGS  "-DLT_LINUX -std=c99 -fmax-errors=1")
add_library(leddar src/LeddarOne.c  src/Modbus.c  src/OS.c)

add_executable(leddar_node src/leddar_node.cpp)
target_link_libraries(leddar_node 
  ${catkin_LIBRARIES}
  leddar
)



