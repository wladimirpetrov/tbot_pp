cmake_minimum_required(VERSION 3.0.2)
project(path_planner_vpetrov) 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Updated executable names to reflect changes
add_executable(
    detection_node
    src/detection_node.cpp
)
target_link_libraries(
    detection_node
    ${catkin_LIBRARIES}
)

add_executable(
    pp_node
    src/pp_node.cpp
)
target_link_libraries(
    pp_node
    ${catkin_LIBRARIES}
)

add_executable(
    node_bot
    src/node_bot.cpp
)
target_link_libraries(
    node_bot
    ${catkin_LIBRARIES}
)

# If definitions.hpp needs to be installed
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

