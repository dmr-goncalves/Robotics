/***************************** Made by Duarte Gonçalves and André Lourenço *********************************/

#include "nxt_maze_solver/MazeSolver.hpp"


int main( int argc, char** argv )
{
    ros::init( argc, argv, "maze_solver_node" );
	MazeSolver mSolver;
	mSolver.run();
	return EXIT_SUCCESS;
}
