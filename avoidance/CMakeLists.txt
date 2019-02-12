cmake_minimum_required(VERSION 2.8.3)
project(avoidance)

add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  mavros
  mavros_extras
  mavros_msgs
  mavlink
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs geometry_msgs mav_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## CMake Setup
# Build in Release mode if nothing is specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(avoidance
		src/CompanionStatus.cpp
)

add_dependencies(avoidance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(avoidance
  ${catkin_LIBRARIES})