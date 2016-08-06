//
// Created by francisco on 10-03-2016.
//

#ifndef TR5_CONTROL_TR5_COMUNICATION_H
#define TR5_CONTROL_TR5_COMUNICATION_H

#include "serial/serial.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"


class Tr5_communication {
private:
    //Pointer to the com port
    serial::Serial serialConn;

    sensor_msgs::JointState jointStatus_;

    sensor_msgs::JointState jointCommand_;

    std::vector<std::string> joint_names_;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_position_command_;
    std::vector<double> joint_effort_command_;
    std::vector<double> joint_velocity_command_;

    std::vector<int> joint_step_command_;



    int n_dof_;


public:
    //Constructor
    Tr5_communication(void);

    Tr5_communication(std::string port, u_int32_t baudrate, int timeout);


    //Destructor
    ~Tr5_communication(void);

    /* Angles in radians */
    int convAngleToSteps(double Angle, int axis);

    double convStepsToAngle(int pos, int axis);

    int degrees_to_steps(double degrees, int axis);

    //Move a single axis
    void moveAxis(int axis, int steps);

    // Move multiple axis, distance by vector
    void moveMultipleAxis(std::vector<int> steps);

    // Initialize Robot
    bool Initialize();

    // get robot message
    char* getMessage(void);

    // close ports
    void close(void);

    //reads robot's axis position
    int axisPosition(int axis);

    //reads multiple axis positions
    sensor_msgs::JointState multipleAxisPosition();

    //move axis with certain speed
    void moveAxisSpeed(int axis, int steps, int time);

    //move multiple axis with multiple speed
    void moveMulAxisSpeed(std::vector<int> steps, int* time);

    int mm_to_steps(double distance);

    //from the joint messages with angles to step to send to the moveMulAxis
    void moveAxisJointState(sensor_msgs::JointState commandMsg);
};


#endif //TR5_CONTROL_TR5_COMUNICATION_H
