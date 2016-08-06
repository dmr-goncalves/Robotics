//
// Created by francisco on 08-03-2016.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial/serial.h"
#include "tr5_communication.h"
#include "sensor_msgs/JointState.h"

#include <sstream>


sensor_msgs::JointState command_;
//int degrees_to_steps(double degrees, int axis)
//{
//    double stepF;
//    int step;
//
//    switch (axis)
//    {
//        case 1:
//            degrees = degrees - 80;
//            stepF = degrees / -0.62745098039215686274509803921569;
//            step = (int)(stepF + 0.5);
//            break;
//        case 2:
//            degrees = 66.6666666666666666666666666666666 - degrees;
//            stepF = degrees / 0.3921568627450980392156862745098;
//            step = (int)(stepF + 0.5);
//            break;
//        case 3:
//            degrees = 0 - degrees;
//            stepF = degrees / 0.3921568627450980392156862745098;
//            step = (int)(stepF + 0.5);
//            break;
//        case 4:
//            degrees = 100 - degrees;
//            stepF = degrees / 0.78431372549019607843137254901961;
//            step = (int)(stepF + 0.5);
//            break;
//        case 5:
//            degrees = 0 - degrees;
//            //degrees = 200 - degrees;
//            stepF = degrees / 0.78431372549019607843137254901961;
//            step = (int)(stepF + 0.5);
//            break;
//        default:
//            break;
//    }
//    if (step > 255){
//        return -1;
//    }
//    else return step;
//}
//
//
//int InitializeRobot(serial::Serial *serialConn)
//{
//    std::cout << "Sending number of bytes " << serialConn->write(" ") << std::endl;
//    usleep(5000);
//    std::string buff;
//    std::cout << "The received buffer is " <<  serialConn->read(buff,1) << std::endl;
//
//    std::cout << buff << std::endl;
//}
//
void move_one_axis(serial::Serial *serialConn, int axis, int steps)
{

    char command1[20] = {  0x08 + axis - 1, steps, 3, '\n', '\r' };
    std::cout << "Sending Command to Robot to move axis "<< axis << " Steps " << steps << std::endl;
    std::cout << "Sending number of bytes " << serialConn->write(command1) << std::endl;
    std::cout << "Sending number of bytes " << serialConn->write(command1) << std::endl;

    usleep(5000);
    //robot will respond with 15
    std::string  buffer1;
    size_t  t = 2;
    std::cout << "robot responded with number of bytes" <<  serialConn->read(buffer1, t) << std::endl;
    std::cout << "The received buffer is " <<  buffer1 << std::endl;
}
//
//int multipleAxisPosition(serial::Serial *serialConn)
//{
//    char command[] = {0x47,3};
//    int* vecPos = new int[6];
//
//    std::cout << "Sending number of bytes " << serialConn->write(command) << std::endl;
//
//    usleep(5000);
//    //robot will respond with 15
//    std::string  buffer1;
//    size_t  t = 2;
//    std::cout << "robot responded with number of bytes" <<  serialConn->read(buffer1, t) << std::endl;
//    std::cout << "The received buffer is " <<  buffer1 << std::endl;
//
//
//    for(int i = 0 ; i < 6; i++)
//    {
//        vecPos[i] = (int)(buffer1[(i+1)]);
//        std::cout << " Pos " << vecPos[i] << std::endl;
//    }
//
//}
//
//void moveMultipleAxis(serial::Serial *serialConn, int* steps)
//{
//    int tam;
//    char Buff[20];
//    char command[]={0xF,steps[0],steps[1],steps[2],steps[3],steps[4],steps[5],3,0};
//    std::cout << "Sending number of bytes " << serialConn->write(command) << std::endl;
//    std::cout << "Sending number of bytes " << serialConn->write(command) << std::endl;
//
//    usleep(5000);
//    //robot will respond with 15
//    std::string  buffer1;
//    size_t  t = 2;
//    std::cout << "robot responded with number of bytes" <<  serialConn->read(buffer1, t) << std::endl;
//    std::cout << "The received buffer is " <<  buffer1 << std::endl;
//}
void commandCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
//    std::cout << "Vector Size " << msg->position.size() << std::endl;
    command_ = *msg;

}





/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

    ros::init(argc, argv, "TR5_Hardware_Communication");

    ros::NodeHandle n;

    sensor_msgs::JointState currentState;

    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("tr5/joints/state", 1000);
    ros::Subscriber sub = n.subscribe("/joint_states", 1000, commandCallback);




    ros::Rate loop_rate(2);


    Tr5_communication Tr5Comm("/dev/ttyUSB0",9600, 256);

    Tr5Comm.Initialize();


    while (ros::ok())
    {
        Tr5Comm.Initialize();
//        currentState = Tr5Comm.multipleAxisPosition();
//        joint_state_pub.publish(currentState);
        Tr5Comm.moveAxisJointState(command_);

        //Tr5Comm.moveAxis(3,0);
        loop_rate.sleep();
        ros::spinOnce();
    }


    return 0;
}