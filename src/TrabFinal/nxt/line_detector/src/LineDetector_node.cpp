/***************************** Made by Duarte Gonçalves and André Lourenço *********************************/

#include "line_detector/LineDetector.hpp"


int main( int argc, char** argv )
{
    ros::init( argc, argv, "line_detector_node" );
	LineDetector lDetector;
	lDetector.run();
	return EXIT_SUCCESS;
}
