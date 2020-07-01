cmake_minimum_required(VERSION 2.8.12)
project(safe_landing_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  tf
  pcl_ros
  mavros
  mavros_extras
  mavros_msgs
  mavlink
  message_generation
  avoidance
)
find_package(PCL 1.7 REQUIRED)

if(DISABLE_SIMULATION)
  message(STATUS "Building avoidance without Gazebo Simulation")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDISABLE_SIMULATION")
endif()

## Generate messages in the 'msg' folder
add_message_files(FILES SLPGridMsg.msg)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

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
  cfg/SafeLandingPlannerNode.cfg
  cfg/WaypointGeneratorNode.cfg
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
  CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs geometry_msgs mav_msgs sensor_msgs message_runtime tf message_runtime avoidance
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
)

## Declare a C++ library
# add_library(avoidance
#   src/${PROJECT_NAME}/avoidance.cpp
# )
set(SAFE_LANDING_PLANNER_CPP_FILES     "src/nodes/safe_landing_planner_node.cpp"
                                       "src/nodes/safe_landing_planner.cpp"
                                       "src/nodes/waypoint_generator.cpp"
                                       "src/nodes/waypoint_generator_node.cpp"
                                       "src/nodes/safe_landing_planner_visualization.cpp"
)

add_library(safe_landing_planner     "${SAFE_LANDING_PLANNER_CPP_FILES}")

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(safe_landing_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(avoidance_node src/avoidance_node.cpp)
add_executable(safe_landing_planner_node src/nodes/safe_landing_planner_node_main.cpp)
add_executable(waypoint_generator_node src/nodes/waypoint_generator_node.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(avoidance_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(avoidance_node
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )
target_link_libraries(safe_landing_planner_node
  safe_landing_planner
  ${catkin_LIBRARIES} )
target_link_libraries(waypoint_generator_node
    safe_landing_planner
    ${catkin_LIBRARIES} )

  #############
  ## Testing ##
  #############

if(CATKIN_ENABLE_TESTING)
    # Add gtest based cpp test target and link libraries
    catkin_add_gtest(${PROJECT_NAME}-test test/main.cpp
                                          test/test_safe_landing_planner.cpp
                                          test/test_waypoint_generator.cpp
                                          test/test_grid.cpp
                                        )

    if(TARGET ${PROJECT_NAME}-test)
        target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}
                                                   ${catkin_LIBRARIES}
                              )
    endif()


    if (${CMAKE_BUILD_TYPE} STREQUAL "Coverage")
        SET(CMAKE_CXX_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage --coverage")
        SET(CMAKE_C_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage --coverage")
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")

        add_custom_target(${PROJECT_NAME}-test_coverage
            COMMAND lcov --zerocounters --directory ${PROJECT_BINARY_DIR}
            COMMAND lcov --capture  --initial --no-external --directory ${PROJECT_BINARY_DIR} --base-directory ${${PROJECT_NAME}_SOURCE_DIR} --output-file base_coverage.info --rc lcov_branch_coverage=1
            COMMAND ${PROJECT_NAME}-test
            COMMAND lcov --capture  --no-external --directory ${PROJECT_BINARY_DIR} --base-directory ${${PROJECT_NAME}_SOURCE_DIR} --output-file test_coverage.info --rc lcov_branch_coverage=1
            COMMAND lcov -a base_coverage.info -a test_coverage.info -o coverage.info --rc lcov_branch_coverage=1
            COMMAND lcov --rc lcov_branch_coverage=1 --summary coverage.info
            WORKING_DIRECTORY .
            DEPENDS ${PROJECT_NAME}-test
        )
        add_custom_target(${PROJECT_NAME}-test_coverage_html
            COMMAND genhtml coverage.info --output-directory out --branch-coverage
            COMMAND x-www-browser out/index.html
            WORKING_DIRECTORY .
            DEPENDS ${PROJECT_NAME}-test_coverage
        )
    endif()

    ## Add folders to be run by python nosetests
    # catkin_add_nosetests(test)
endif()
