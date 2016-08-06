/***************************** Made by Duarte Gonçalves and André Lourenço *********************************/

#include "tr5_robot_controller/Controller.hpp"


int main( int argc, char** argv )
{
    ros::init( argc, argv, "tr5_robot_controller_node" );

    TR5::Controller arm_manipulator;

    arm_manipulator.run();

    return 0;
}
