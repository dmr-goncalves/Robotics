cmake_minimum_required(VERSION 2.8.3)
project(motor_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)

catkin_package(INCLUDE_DIRS include LIBRARIES motor_controller CATKIN_DEPENDS geometry_msgs roscpp  DEPENDS system_lib)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable( motor_controller_node src/MotorController_node.cpp src/MotorController.cpp)

add_dependencies( motor_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries( motor_controller_node ${catkin_LIBRARIES} )
