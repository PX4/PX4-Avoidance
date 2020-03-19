cmake_minimum_required(VERSION 3.5)
project(local_planner)

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
#find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PythonInterp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

if(DISABLE_SIMULATION)
  message(STATUS "Building avoidance without Gazebo Simulation")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDISABLE_SIMULATION")
endif()

#################
# Setup targets #
#################

## CMake Setup
# Build in Release mode if nothing is specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

## Declare a C++ library
set(LOCAL_PLANNER_CPP_FILES
  "src/nodes/local_planner.cpp"
  "src/nodes/waypoint_generator.cpp"
  "src/nodes/tree_node.cpp"
  "src/nodes/star_planner.cpp"
  "src/nodes/planner_functions.cpp"
  "src/nodes/local_planner_visualization.cpp"
  "src/utils/trajectory_simulator.cpp"
)

# Add local_planner lib
add_library(local_planner SHARED "${LOCAL_PLANNER_CPP_FILES}")
ament_target_dependencies(local_planner Eigen3 px4_msgs ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_include_directories(local_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(local_planner Eigen3::Eigen)

## Declare a C++ executable
add_executable(local_planner_node
  src/nodes/local_planner_node_main.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  local_planner_node
  local_planner
  #avoidance
  ${YAML_CPP_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Add node dependencies
ament_target_dependencies(local_planner_node local_planner px4_msgs rclcpp) #avoidance)

# Export information to downstream packages
ament_export_dependencies(ament_cmake local_planner rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3 px4_msgs geometry_msgs sensor_msgs std_msgs)
ament_export_interfaces(export_local_planner HAS_LIBRARY_TARGET)
ament_export_include_directories(include ${px4_msgs_INCLUDE_DIRS})
ament_export_libraries()

#############
## Install ##
#############

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
)

# Install artifacts
install(DIRECTORY cmake
        DESTINATION share/${PROJECT_NAME}
)
install(TARGETS local_planner
        EXPORT export_local_planner
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
install(TARGETS local_planner_node
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

# if(CATKIN_ENABLE_TESTING)
#     # Add gtest based cpp test target and link libraries
#     catkin_add_gtest(${PROJECT_NAME}-test test/main.cpp
#                                           test/test_example.cpp
#                                           test/test_local_planner.cpp
#                                           test/test_planner_functions.cpp
#                                           test/test_star_planner.cpp
#                                           test/test_trajectory_simulator.cpp
#                                           test/test_waypoint_generator.cpp)
#
#     catkin_add_gtest(${PROJECT_NAME}-test-roscore test/main.cpp
#                                         test/test_local_planner_nodelet.cpp)
#     if(TARGET ${PROJECT_NAME}-test)
#         target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}
#                                                  ${catkin_LIBRARIES}
#                                                  ${YAML_CPP_LIBRARIES})
#     endif()
#
#     if(TARGET ${PROJECT_NAME}-test-roscore)
#         target_link_libraries(${PROJECT_NAME}-test-roscore local_planner_nodelet
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
#             COMMAND ${PROJECT_NAME}-test-roscore
#             COMMAND lcov --capture  --no-external --directory ${PROJECT_BINARY_DIR} --base-directory ${${PROJECT_NAME}_SOURCE_DIR} --output-file test_coverage.info --rc lcov_branch_coverage=1
#             COMMAND lcov -a base_coverage.info -a test_coverage.info -o coverage.info --rc lcov_branch_coverage=1
#             COMMAND lcov --rc lcov_branch_coverage=1 --summary coverage.info
#             WORKING_DIRECTORY .
#             DEPENDS ${PROJECT_NAME}-test ${PROJECT_NAME}-test-roscore
#         )
#         add_custom_target(${PROJECT_NAME}-test_coverage_html
#             COMMAND genhtml coverage.info --output-directory out --branch-coverage
#             COMMAND x-www-browser out/index.html
#             WORKING_DIRECTORY .
#             DEPENDS ${PROJECT_NAME}-test_coverage
#         )
#     endif()
#
#     ## Add folders to be run by python nosetests
#     # catkin_add_nosetests(test)
# endif()

ament_package()
