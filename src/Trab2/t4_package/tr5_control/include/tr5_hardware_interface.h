//
// Created by francisco on 08-03-2016.
//

#ifndef TR5_CONTROL_TR5_HARDWARE_INTERFACE_H
#define TR5_CONTROL_TR5_HARDWARE_INTERFACE_H


// ROS
#include <trajectory_msgs/JointTrajectory.h>


// Parent class
#include <tr5_interface.h>
//Communication
#include "serial/serial.h"


namespace tr5_control
{

    static const double STATE_EXPIRED_TIMEOUT = 2.0;

    class Tr5HardwareInterface : public Tr5Interface
    {
    private:

        // Publishers
        ros::Publisher pub_joint_command_;
        ros::Publisher pub_trajectory_command_;

        // Subscriber
        ros::Subscriber cuff_squeezed_sub_; // this is used to update the controllers when manual mode is started

        // Messages to send
        trajectory_msgs::JointTrajectory trajectory_command_msg_;
        sensor_msgs::JointState output_msg_;

        // Track button status
        bool cuff_squeezed_previous;

        // Convert a joint states message to our ids
        std::vector<int> joint_id_to_joint_states_id_;

        serial::Serial serialConn;

        ros::NodeHandle public_node_handle_;






    public:

        /**
         * \brief Constructor/Descructor
         */
        Tr5HardwareInterface(const std::string &arm_name, double loop_hz);
        ~Tr5HardwareInterface();

        /**
         * \brief Initialice hardware interface
         * \return false if an error occurred during initialization
         */
        bool init(
                hardware_interface::JointStateInterface&    js_interface,
                hardware_interface::EffortJointInterface&   ej_interface,
                hardware_interface::VelocityJointInterface& vj_interface,
                hardware_interface::PositionJointInterface& pj_interface,
                int* joint_mode,
                sensor_msgs::JointStateConstPtr state_msg
        );

        /**
         * \brief Buffers joint state info from Baxter ROS topic
         * \param
         */
        void stateCallback(const sensor_msgs::JointStateConstPtr& msg);

        /**
         * \brief Copy the joint state message into our hardware interface datastructures
         */
        void read( sensor_msgs::JointStateConstPtr &state_msg );

        /**
         * \brief Publish our hardware interface datastructures commands to Baxter hardware
         */
        void write(ros::Duration elapsed_time);


        /**
         * \Initialize robot
         */
        void robotInitialization(void);


        /**
         * \brief This is called when Baxter is disabled, so that we can update the desired positions
         */
        void robotDisabledCallback();

        /**
         * \brief inform the trajectory controller to update its setpoint
         */
        void publishCurrentLocation();
    };

    void mainNodeThread(void);

} // namespace

#endif //TR5_CONTROL_TR5_HARDWARE_INTERFACE_H
