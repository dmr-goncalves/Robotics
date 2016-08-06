cmake_minimum_required(VERSION 2.8.3)
project(obstacle_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  nxt_msgs
  roscpp
  tf
)

catkin_package(INCLUDE_DIRS include LIBRARIES obstacle_detector CATKIN_DEPENDS nav_msgs nxt_msgs DEPENDS system_lib)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
## Declare a C++ executable
add_executable(obstacle_detector_node src/ObstacleDetector_node.cpp src/ObstacleDetector.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(obstacle_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(obstacle_detector_node
  ${catkin_LIBRARIES}
)
