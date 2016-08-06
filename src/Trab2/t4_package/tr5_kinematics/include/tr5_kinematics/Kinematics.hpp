/***************************** Made by Duarte Gonçalves and André Lourenço *********************************/

#ifndef _TR5_KINEMATICS_HPP__
#define _TR5_KINEMATICS_HPP__

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <tr5_kinematics/DoForwardKinematic.h>
#include <tr5_kinematics/DoInverseKinematic.h>

namespace TR5
{
    class Kinematics
    {
        public:

            Kinematics();
            virtual ~Kinematics(){};

            void run();

        private:

            ros::NodeHandle nh;
            ros::ServiceServer                            m_pub_do_fk;///do_fk
            ros::ServiceServer                            m_pub_do_ik; ///do_ik
            float                                         liftJointHeight;
        	float                                         upperArmLenght;
        	float                                         forearmLenght;
        	float                                         endEffectorLenght;
            bool do_fk (tr5_kinematics::DoForwardKinematic::Request  &req, tr5_kinematics::DoForwardKinematic::Response &res );
            bool do_ik (tr5_kinematics::DoInverseKinematic::Request  &req, tr5_kinematics::DoInverseKinematic::Response &res );
            double degrees(double radians);
            double radians(double degrees);
    };
}

#endif /* _TR5_KINEMATICS_HPP__ */
