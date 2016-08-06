cmake_minimum_required(VERSION 2.8.3)
project(magnetic_detector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nxt_msgs
  roscpp
  visualization_msgs
  nav_msgs
)


catkin_package(INCLUDE_DIRS include LIBRARIES magnetic_detector CATKIN_DEPENDS geometry_msgs nav_msgs roscpp nxt_msgs DEPENDS system_lib)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(magnetic_detector_node src/MagneticDetector_node.cpp src/MagneticDetector.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(magnetic_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(magnetic_detector_node
   ${catkin_LIBRARIES}
)
