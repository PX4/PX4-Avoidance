cmake_minimum_required(VERSION 2.8.3)
project(local_planner)

add_definitions(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  tf
  pcl_ros
  mavros
  mavros_extras
  mavros_msgs
  mavlink
)
find_package(PCL 1.7 REQUIRED)




################################################
## Gazebo Simulation
################################################
# Disable yaml-cpp when no simulation is required
if(DISABLE_SIMULATION)
  message(STATUS "Building local planner without Gazebo Simulation")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDISABLE_SIMULATION")
else()
  message(STATUS "Building local planner with Gazebo Simulation")
  find_package(yaml-cpp REQUIRED)
endif(DISABLE_SIMULATION)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/local_planner_node.cfg
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
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs geometry_msgs mav_msgs sensor_msgs message_runtime tf
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
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

## Declare a C++ library
# add_library(avoidance
#   src/${PROJECT_NAME}/avoidance.cpp
# )

if(DISABLE_SIMULATION)
  add_library(local_planner
    src/nodes/local_planner.cpp
    src/nodes/waypoint_generator.cpp
    src/nodes/histogram.cpp
    src/nodes/tree_node.cpp
    src/nodes/box.cpp
    src/nodes/star_planner.cpp
    src/nodes/planner_functions.cpp
    src/nodes/common.cpp
  )
else()
  message(STATUS "Building local planner with Gazebo Simulation")
  add_library(local_planner
    src/nodes/local_planner.cpp
    src/nodes/waypoint_generator.cpp
    src/nodes/histogram.cpp
    src/nodes/tree_node.cpp
    src/nodes/box.cpp
    src/nodes/star_planner.cpp
    src/nodes/planner_functions.cpp
    src/nodes/common.cpp
    src/nodes/rviz_world_loader.cpp
  )
endif(DISABLE_SIMULATION)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(avoidance_node src/avoidance_node.cpp)
add_executable(local_planner_node src/nodes/local_planner_node.cpp
                                  src/nodes/local_planner_node_main.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(avoidance_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(avoidance_node
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )
target_link_libraries(local_planner_node
  local_planner
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS avoidance avoidance_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
	# Add gtest based cpp test target and link libraries
	catkin_add_gtest(${PROJECT_NAME}-test test/main.cpp
	                                      test/test_example.cpp
	                                      test/test_common.cpp
	                                      test/test_local_planner.cpp
	                                      test/test_planner_functions.cpp
                                              test/test_star_planner.cpp
                                              test/test_waypoint_generator.cpp)
	if(TARGET ${PROJECT_NAME}-test)
	  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}
	                                             ${catkin_LIBRARIES}
	                                             ${YAML_CPP_LIBRARIES})
	endif()

	## Add folders to be run by python nosetests
	# catkin_add_nosetests(test)
endif()
