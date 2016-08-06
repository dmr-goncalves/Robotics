cmake_minimum_required(VERSION 2.8.3)
project(nxt_maze_solver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  sensor_msgs
  tf
)


catkin_package(INCLUDE_DIRS include LIBRARIES nxt_maze_solver CATKIN_DEPENDS std_msgs DEPENDS system_lib)

include_directories( include ${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(maze_solver_node src/MazeSolver_node.cpp src/MazeSolver.cpp )

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(maze_solver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

## Specify libraries to link a library or executable target against
target_link_libraries(maze_solver_node
   ${catkin_LIBRARIES}
)
