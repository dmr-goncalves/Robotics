//
// Created by francisco on 11-03-2016.
//

#ifndef TR5_CONTROL_TR5_CONTROL_H
#define TR5_CONTROL_TR5_CONTROL_H

// Boost
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>

// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
//#include <hardware_interface/joint_mode_interface.h>
#include <hardware_interface/robot_hw.h>

#include <tr5_interface.h>
#include <tr5_hardware_interface.h>
namespace tr5_control
{

            static const double NUM_JOINTS = 7;

            class TRHardwareInterface : public hardware_interface::RobotHW
            {
            private:

                // Node Handles
                ros::NodeHandle nh_; // no namespace

                // Timing
                ros::Duration control_period_;
                ros::Time last_sim_time_ros_;
                ros::Duration elapsed_time_;
                double loop_hz_;

                // Interfaces
                hardware_interface::JointStateInterface    js_interface_;
    //            hardware_interface::JointModeInterface     jm_interface_;
                hardware_interface::EffortJointInterface   ej_interface_;
                hardware_interface::VelocityJointInterface vj_interface_;
                hardware_interface::PositionJointInterface pj_interface_;


                // sub-hardware interfaces
                Tr5HardwareInterface *arm_hw_;



                boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

                ros::Timer non_realtime_loop_;

                bool in_simulation_;

                // Which joint mode are we in
                int joint_mode_;

                // Buffer of joint states to share between arms
                sensor_msgs::JointStateConstPtr state_msg_;
                ros::Time state_msg_timestamp_;

                // Subscriber
                ros::Subscriber sub_joint_state_;

            public:

                /**
                 * \brief Constructor/Descructor
                 */
                TRHardwareInterface(bool in_simulation);
                ~TRHardwareInterface();

                /**
                 * \brief Checks if the state message from Baxter is out of date
                 * \return true if expired
                 */
                bool stateExpired();

                void stateCallback(const sensor_msgs::JointStateConstPtr& msg);

                void update(const ros::TimerEvent& e);

            };

}; // namespace






#endif //TR5_CONTROL_TR5_CONTROL_H
