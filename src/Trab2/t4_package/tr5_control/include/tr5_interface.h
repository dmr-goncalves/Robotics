//
// Created by francisco on 08-03-2016.
//

#ifndef TR5_CONTROL_TR5_INTERFACE_H
#define TR5_CONTROL_TR5_INTERFACE_H

// Boost
#include <boost/shared_ptr.hpp>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// ros_control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
//#include <hardware_interface/joint_mode_interface.h>

// Baxter
//#include <baxter_core_msgs/JointCommand.h>
//#include <baxter_core_msgs/JointCommand.h>

namespace tr5_control
{

    enum TR5ControlMode { POSITION, VELOCITY, TORQUE };

    class Tr5Interface
    {
    protected:

        // Node Handles
        ros::NodeHandle nh_; // no namespace

        // Number of joints we are using
        unsigned int n_dof_;

        std::vector<std::string> joint_names_;
        std::vector<double> joint_position_;
        std::vector<double> joint_velocity_;
        std::vector<double> joint_effort_;
        std::vector<double> joint_position_command_;
        std::vector<double> joint_effort_command_;
        std::vector<double> joint_velocity_command_;

        // Track current hardware interface mode we are in
        int* joint_mode_;

        // Speed of hardware loop
        double loop_hz_;

        // Name of this arm
        std::string arm_name_;

    public:

        /**
         * \brief Constructor/Descructor
         */
        Tr5Interface(const std::string &arm_name, double loop_hz)
                : arm_name_(arm_name),
                  loop_hz_(loop_hz)
        {};

        ~Tr5Interface()
        {};

        /**
         * \brief Initialice hardware interface
         * \return false if an error occurred during initialization
         */
        virtual bool init(
                hardware_interface::JointStateInterface&    js_interface,
                hardware_interface::EffortJointInterface&   ej_interface,
                hardware_interface::VelocityJointInterface& vj_interface,
                hardware_interface::PositionJointInterface& pj_interface,
                int* joint_mode,
                sensor_msgs::JointStateConstPtr state_msg
        )
        { return true; };

        /**
         * \brief Copy the joint state message into our hardware interface datastructures
         */
        virtual void read( sensor_msgs::JointStateConstPtr &state_msg )
        {};

        /**
         * \brief Publish our hardware interface datastructures commands to Baxter hardware
         */
        virtual void write(ros::Duration elapsed_time)
        {};

        /**
         * \brief This is called when tr5 is disabled, so that we can update the desired positions
         */
        virtual void robotDisabledCallback()
        {};

    };

/*    typedef boost::shared_ptr<baxter_control::ArmInterface> ArmInterfacePtr;
    typedef boost::shared_ptr<const baxter_control::ArmInterface> ArmInterfaceConstPtr;*/

} // namespace


#endif //TR5_CONTROL_TR5_HARDWARE_INTERFACE_H
