cmake_minimum_required(VERSION 2.8.3)
project(neighbor_sense)

find_package(catkin REQUIRED COMPONENTS
  apriltags_ros
  roscpp
  rospy
  std_msgs
)


## Generate added messages and services with any dependencies listed here#
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  apriltags_ros
#)

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
 INCLUDE_DIRS include
#  LIBRARIES neighbor_sense
#  CATKIN_DEPENDS apriltags_ros roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(neighbor_sense ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(neighbor_sense src/neighbor_sense.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(neighbor_sense ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(neighbor_sense
  ${catkin_LIBRARIES}
)
