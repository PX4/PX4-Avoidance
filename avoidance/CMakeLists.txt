cmake_minimum_required(VERSION 2.8.12)
project(avoidance)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
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

if(DISABLE_SIMULATION)
  message(STATUS "Building avoidance without Gazebo Simulation")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDISABLE_SIMULATION")
else()
  message(STATUS "Building avoidance with Gazebo Simulation")
  find_package(yaml-cpp REQUIRED)
endif(DISABLE_SIMULATION)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs geometry_msgs sensor_msgs message_runtime tf
  LIBRARIES avoidance
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
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

## Declare a C++ library
set(AVOIDANCE_CPP_FILES   "src/common.cpp"
                          "src/histogram.cpp"
                          "src/transform_buffer.cpp"
                          "src/avoidance_node.cpp"
)
if(NOT DISABLE_SIMULATION)
  set(AVOIDANCE_CPP_FILES "${AVOIDANCE_CPP_FILES}"
                              "src/rviz_world_loader.cpp")
endif()
add_library(avoidance     "${AVOIDANCE_CPP_FILES}")


## Add cmake target dependencies of the library
add_dependencies(avoidance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(avoidance
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
    # Add gtest based cpp test target and link libraries
    catkin_add_gtest(${PROJECT_NAME}-test test/main.cpp
                                          test/test_common.cpp
                                          test/test_usm.cpp
                                          test/test_transform_buffer.cpp
                    )

    if(TARGET ${PROJECT_NAME}-test)
      target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}
                                                 ${catkin_LIBRARIES}
                                                 ${YAML_CPP_LIBRARIES})
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
endif()
