cmake_minimum_required(VERSION 2.8.3)
project(tr5_robot_controller)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp rospy sensor_msgs interactive_markers visualization_msgs message_generation)
find_package(tr5_kinematics)
################################################
## Declare ROS messages, services and actions ##
################################################


## Generate services in the 'srv' folder
add_service_files(FILES KinematicMode.srv )


## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES std_msgs sensor_msgs geometry_msgs)


catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS  tr5_kinematics roscpp std_msgs geometry_msgs sensor_msgs message_runtime DEPENDS system_lib)

include_directories(include ${catkin_INCLUDE_DIRS} ${tr5_kinematics_INCLUDE_DIRS})

add_executable( tr5_robot_controller_node src/tr5_robot_controller_node.cpp src/Controller.cpp)

add_dependencies( tr5_robot_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${tr5_kinematics_EXPORTED_TARGETS})

target_link_libraries( tr5_robot_controller_node ${catkin_LIBRARIES} ${tr5_kinematics_LIBRARIES})
