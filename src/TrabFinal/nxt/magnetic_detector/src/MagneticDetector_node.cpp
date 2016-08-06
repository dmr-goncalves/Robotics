/***************************** Made by Duarte Gonçalves and André Lourenço *********************************/

#include "magnetic_detector/MagneticDetector.hpp"


int main( int argc, char** argv )
{
    ros::init( argc, argv, "magnetic_detector_node" );
	MagneticDetector mDetector;
	mDetector.run();
	return EXIT_SUCCESS;
}
