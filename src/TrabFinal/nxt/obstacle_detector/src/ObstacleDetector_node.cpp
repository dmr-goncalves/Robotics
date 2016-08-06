/***************************** Made by Duarte Gonçalves and André Lourenço *********************************/

#include "obstacle_detector/ObstacleDetector.hpp"


int main( int argc, char** argv )
{
    ros::init( argc, argv, "obstacle_detector_node" );
	ObstacleDetector oDetector;
	oDetector.run();
	return EXIT_SUCCESS;
}
