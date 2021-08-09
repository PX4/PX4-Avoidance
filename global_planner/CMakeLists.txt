cmake_minimum_required(VERSION 3.5)
project(global_planner)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Disable Wredundant-decls warnings since rosidl generates redundant function declarations 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1z")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(avoidance REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PathWithRiskMsg.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

# find_package PCL interferes with message generation(https://github.com/ros2/rosidl/issues/402). It should be below of rosidl_generate_interfaces

find_package(PCL 1.8 REQUIRED)

set(DISABLE_SIMULATION OFF)

if(DISABLE_SIMULATION)
  message(STATUS "Building avoidance without Gazebo Simulation")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDISABLE_SIMULATION")
endif()

include_directories(
  include
  ${avoidance_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${octomap_msgs_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_sensor_msgs_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  src/library
  src/nodes)

link_directories(${Boost_LIBRARY_DIRS})

set(GLOBAL_PLANNER_CPP_FILES
  "src/library/cell.cpp"
  "src/library/global_planner.cpp"
  "src/library/node.cpp"
  "src/nodes/global_planner_node.cpp"
  "src/nodes/global_planner_node_main.cpp"
)

add_executable(global_planner_node
  ${GLOBAL_PLANNER_CPP_FILES})

target_link_libraries(global_planner_node
  ${Boost_LIBRARIES}
  ${tf2_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

ament_target_dependencies(global_planner_node
  rclcpp
  avoidance
  px4_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  octomap
  octomap_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_kdl
  tf2_sensor_msgs
  tf2_geometry_msgs
  PCL)

rosidl_target_interfaces(global_planner_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
)

# Install artifacts
install(TARGETS global_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY
  resource
  DESTINATION share/${PROJECT_NAME})

install(TARGETS global_planner_node
  RUNTIME DESTINATION bin
)

add_library(avoidance SHARED ${GLOBAL_PLANNER_CPP_FILES})

ament_export_libraries(avoidance)

ament_package()

