cmake_minimum_required(VERSION 2.8.3)
project(line_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS nxt_msgs roscpp std_msgs)

catkin_package(INCLUDE_DIRS include LIBRARIES line_detector CATKIN_DEPENDS nxt_msgs roscpp std_msgs DEPENDS system_lib)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(line_detector_node src/LineDetector_node.cpp src/LineDetector.cpp )

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(line_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(line_detector_node
   ${catkin_LIBRARIES}
)
