cmake_minimum_required(VERSION 2.8.3)
project(tr5_kinematics)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs geometry_msgs message_generation)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(FILES DoForwardKinematic.srv DoInverseKinematic.srv)

## Generate added messages and services with any dependencies listed here
generate_messages( DEPENDENCIES std_msgs sensor_msgs geometry_msgs)


catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs message_runtime DEPENDS system_lib)

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable(tr5_kinematics_node src/tr5_kinematics_node.cpp src/Kinematics.cpp )

add_dependencies( tr5_kinematics_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

add_library(${PROJECT_NAME} src/tr5_kinematics_node.cpp src/Kinematics.cpp )

target_link_libraries( tr5_kinematics_node ${catkin_LIBRARIES} )
