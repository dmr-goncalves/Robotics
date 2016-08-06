/***************************** Made by Duarte Gonçalves and André Lourenço *********************************/

#include "motor_controller/MotorController.hpp"


int main( int argc, char** argv )
{
    ros::init( argc, argv, "motor_controller_node" );
	MotorController ctrl;
	ctrl.run();
	return EXIT_SUCCESS;
}
