/***************************** Made by Duarte Gonçalves and André Lourenço *********************************/

#include "tr5_kinematics/Kinematics.hpp"


int main( int argc, char** argv )
{
    ros::init( argc, argv, "tr5_kinematics_node" );
	TR5::Kinematics kin;
	kin.run();
	return EXIT_SUCCESS;
}
