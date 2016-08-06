/***************************** Made by Duarte Gonçalves and André Lourenço *********************************/
#ifndef _MOTOR_CONTROLLER_HPP__
#define _MOTOR_CONTROLLER_HPP__

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

class MotorController
    {
        public:

            MotorController();
            virtual ~MotorController(){};

            void run();

        private:

            ros::NodeHandle                               nh1;
            ros::Publisher                                m_pub_jtCommand; // /nxt/cmd_vel
            ros::Subscriber                               m_sub_moveNXT; // /nxt/moveNXT
            float                                         linearVelocity;
            float                                         angularVelocity;
            float                                         linearEffort;
            float                                         angularEffort;
            float                                         wheelRadius;
            void moveNXTClbk ( const geometry_msgs::Twist::ConstPtr& msg);
            float convertVel2Effort(float velocity);


};

#endif /* _MOTOR_CONTROLLER_HPP__ */
