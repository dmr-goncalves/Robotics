cmake_minimum_required(VERSION 2.8.3)
project(tr5_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        #moveit_ros_planning_interface
        std_msgs
        roscpp
        controller_manager
        transmission_interface
        hardware_interface
        sensor_msgs
        joint_limits_interface
        trajectory_msgs
        moveit_msgs
        moveit_visual_tools
        serial
        )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES
        tr5_control
        CATKIN_DEPENDS
        moveit_ros_planning_interface
        std_msgs
        roscpp
        controller_manager
        transmission_interface
        hardware_interface
        sensor_msgs
        joint_limits_interface
        trajectory_msgs
        moveit_msgs
        moveit_visual_tools
        serial
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(tr5_communication src/tr5_communication.cpp)
target_link_libraries(tr5_communication
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )

add_dependencies(tr5_communication ${catkin_EXPORTED_TARGETS}) # don't build until necessary msgs are finish

add_library(tr5_hardware_interface src/tr5_hardware_interface.cpp)
target_link_libraries(tr5_hardware_interface
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )
add_dependencies(tr5_hardware_interface ${catkin_EXPORTED_TARGETS}) # don't build until necessary msgs are finish


add_executable(tr5_control src/tr5_control.cpp)

target_link_libraries(tr5_control
        tr5_hardware_interface
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )
add_dependencies(tr5_control ${catkin_EXPORTED_TARGETS}) # don't build until necessary msgs are finish


add_executable(tr5_comm_node src/tr5_comm_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(tr5_comm_node
        tr5_communication
        ${catkin_LIBRARIES}
        )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS tr5_control tr5_control_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tr5_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
