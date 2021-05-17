cmake_minimum_required(VERSION 3.5)
project(avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(perception_pcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)

get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PythonInterp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.8 REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})

if(DISABLE_SIMULATION)
  message(STATUS "Building avoidance without Gazebo Simulation")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDISABLE_SIMULATION")
else()
  message(STATUS "Building avoidance with Gazebo Simulation")
  find_package(yaml-cpp REQUIRED)
endif(DISABLE_SIMULATION)

#################
# Setup targets #
#################

## CMake Setup
# Build in Release mode if nothing is specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

## Specify additional locations of header files
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

## Declare a C++ library
set(AVOIDANCE_CPP_FILES
  "src/common.cpp"
  "src/histogram.cpp"
  "src/transform_buffer.cpp"
  "src/avoidance_node.cpp"
)
if(NOT DISABLE_SIMULATION)
  set(AVOIDANCE_CPP_FILES "${AVOIDANCE_CPP_FILES}" "src/rviz_world_loader.cpp")
endif()

# Add avoidance lib
add_library(avoidance SHARED "${AVOIDANCE_CPP_FILES}")
ament_target_dependencies(avoidance Eigen3 px4_msgs PCL ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_include_directories(avoidance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Eigen3_INCLUDE_DIRS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  avoidance
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# Add node dependencies
ament_target_dependencies(avoidance px4_msgs rclcpp Eigen3)

# Export information to downstream packages
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs std_msgs)
ament_export_targets(export_avoidance HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(avoidance)

#############
## Install ##
#############

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)

# Install artifacts
install(TARGETS avoidance
        EXPORT export_avoidance
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

# Install launch files
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
	sim
	DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

# if(CATKIN_ENABLE_TESTING)
#     # Add gtest based cpp test target and link libraries
#     catkin_add_gtest(${PROJECT_NAME}-test test/main.cpp
#                                           test/test_common.cpp
#                                           test/test_usm.cpp
#                                           test/test_transform_buffer.cpp
#                     )
#
#     if(TARGET ${PROJECT_NAME}-test)
#       target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}
#                                                  ${catkin_LIBRARIES}
#                                                  ${YAML_CPP_LIBRARIES})
#     endif()
#
#
#     if (${CMAKE_BUILD_TYPE} STREQUAL "Coverage")
#         SET(CMAKE_CXX_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage --coverage")
#         SET(CMAKE_C_FLAGS "-g -O0 -fprofile-arcs -ftest-coverage --coverage")
#         SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
#
#         add_custom_target(${PROJECT_NAME}-test_coverage
#             COMMAND lcov --zerocounters --directory ${PROJECT_BINARY_DIR}
#             COMMAND lcov --capture  --initial --no-external --directory ${PROJECT_BINARY_DIR} --base-directory ${${PROJECT_NAME}_SOURCE_DIR} --output-file base_coverage.info --rc lcov_branch_coverage=1
#             COMMAND ${PROJECT_NAME}-test
#             COMMAND lcov --capture  --no-external --directory ${PROJECT_BINARY_DIR} --base-directory ${${PROJECT_NAME}_SOURCE_DIR} --output-file test_coverage.info --rc lcov_branch_coverage=1
#             COMMAND lcov -a base_coverage.info -a test_coverage.info -o coverage.info --rc lcov_branch_coverage=1
#             COMMAND lcov --rc lcov_branch_coverage=1 --summary coverage.info
#             WORKING_DIRECTORY .
#             DEPENDS ${PROJECT_NAME}-test
#         )
#         add_custom_target(${PROJECT_NAME}-test_coverage_html
#             COMMAND genhtml coverage.info --output-directory out --branch-coverage
#             COMMAND x-www-browser out/index.html
#             WORKING_DIRECTORY .
#             DEPENDS ${PROJECT_NAME}-test_coverage
#         )
#     endif()
# endif()

ament_package()
