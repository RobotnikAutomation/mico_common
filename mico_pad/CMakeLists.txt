cmake_minimum_required(VERSION 2.8.3)
project(mico_pad)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
#  jaco_msgs
  kinova_msgs
  actionlib
  message_generation
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(mico_pad src/mico_pad.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(mico_pad kinova_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(mico_pad
   ${catkin_LIBRARIES}
)
