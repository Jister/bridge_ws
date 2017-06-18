cmake_minimum_required(VERSION 2.8.3)
project(laserscan)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  laser_geometry
)
find_package(Eigen3 REQUIRED)

add_message_files(
   FILES
   Laser.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES laserscan
#  CATKIN_DEPENDS roscpp rospy std_msg
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(laserscan
#   src/${PROJECT_NAME}/laserscan.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(laserscan ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(scan_to_cloud 
  src/scan_to_cloud.cpp
)
add_executable(find_min_distance 
  src/find_min_distance.cpp
)
add_executable(find_position
  src/find_position.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(laserscan_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(scan_to_cloud
  ${catkin_LIBRARIES}
)

target_link_libraries(find_min_distance 
  ${catkin_LIBRARIES}
)

target_link_libraries(find_position
  ${catkin_LIBRARIES}
)


