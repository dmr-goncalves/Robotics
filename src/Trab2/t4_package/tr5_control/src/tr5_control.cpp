//
// Created by francisco on 11-03-2016.
//

#include "tr5_control.h"

namespace tr5_control
{

    TRHardwareInterface::TRHardwareInterface(bool in_simulation)
            : in_simulation_(in_simulation),
              joint_mode_(1),
              loop_hz_(100)
    {
        if( in_simulation_ )
        {
            ROS_INFO_STREAM_NAMED("hardware_interface","Running in simulation mode");
//            right_arm_hw_.reset(new tr5_control::ArmSimulatorInterface("right",loop_hz_));
//            left_arm_hw_.reset(new tr5_control::ArmSimulatorInterface("left",loop_hz_));
        }
        else
        {
            ROS_INFO_STREAM_NAMED("hardware_interface","Running in hardware mode");
            arm_hw_ = (new tr5_control::Tr5HardwareInterface("tr5",loop_hz_));
        }

        // Set the joint mode interface data
      //  jm_interface_.registerHandle(hardware_interface::JointModeHandle("joint_mode", &joint_mode_));

        // Start the shared joint state subscriber
        sub_joint_state_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1,
                                                                  &TRHardwareInterface::stateCallback, this);

        // Wait for first state message to be recieved if we are not in simulation
        if (!in_simulation_)
        {
            // Loop until we find a joint_state message from Baxter
            do
            {
                // Loop until we get our first joint_state message
                while(ros::ok() && state_msg_timestamp_.toSec() == 0)
                {
                    ROS_INFO_STREAM_NAMED("hardware_interface","Waiting for first state message to be received");
                    ros::spinOnce();
                    ros::Duration(0.25).sleep();
                }
            } while (state_msg_->name.size() != NUM_JOINTS);
        }

        // Initialize arm
        arm_hw_->init(js_interface_, ej_interface_, vj_interface_, pj_interface_, &joint_mode_, state_msg_);

        // Register interfaces
        registerInterface(&js_interface_);
//        registerInterface(&ej_interface_);
        registerInterface(&vj_interface_);
        registerInterface(&pj_interface_);

        // Create the controller manager
        ROS_DEBUG_STREAM_NAMED("hardware_interface","Loading controller_manager");
        controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
//
        ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
        non_realtime_loop_ = nh_.createTimer(update_freq, &TRHardwareInterface::update, this);
//
        ROS_INFO_NAMED("hardware_interface", "Loaded tr_hardware_interface.");
    }

    TRHardwareInterface::~TRHardwareInterface()
    {
        //baxter_util_.disableBaxter();
    }

    bool TRHardwareInterface::stateExpired()
    {
        // Check that we have a non-expired state message
        // \todo lower the expiration duration
        if( ros::Time::now() > state_msg_timestamp_ + ros::Duration(STATE_EXPIRED_TIMEOUT)) // check that the message timestamp is no older than 1 second
        {

            ROS_WARN_STREAM_THROTTLE_NAMED(1,"hardware_interface","State expired. Last received state " << (ros::Time::now() - state_msg_timestamp_).toSec() << " seconds ago." );
            return true;
        }
        return false;
    }

    void TRHardwareInterface::stateCallback(const sensor_msgs::JointStateConstPtr& msg)
    {
        // Check if this message has the correct number of joints
        if( msg->name.size() != NUM_JOINTS )
        {
            return;
        }

        // Copy the latest message into a buffer
        state_msg_ = msg;
        state_msg_timestamp_ = ros::Time::now();
    }

    void TRHardwareInterface::update(const ros::TimerEvent& e)
    {

        // Check if state msg from Baxter is expired
        if( !in_simulation_ && stateExpired() )
            return;

        elapsed_time_ = ros::Duration(e.current_real - e.last_real);

        // Input
        arm_hw_->read(state_msg_);

        // Control
        controller_manager_->update(ros::Time::now(), elapsed_time_);

        // Output
        arm_hw_->write(elapsed_time_);
    }


}// namespace

int main(int argc, char** argv)
{
    ROS_INFO_STREAM_NAMED("hardware_interface","Starting hardware interface...");

    ros::init(argc, argv, "tr5_hardware_interface");

    // Allow the action server to receive and send ros messages
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle nh;

    bool in_simulation = false;

    // Parse command line arguments
    for (std::size_t i = 0; i < argc; ++i)
    {
        if( std::string(argv[i]).compare("--simulation") == 0 )
        {
            ROS_INFO_STREAM_NAMED("main","Baxter Hardware Interface in simulation mode");
            in_simulation = true;
        }
    }

    tr5_control::TRHardwareInterface tr5_1(in_simulation);

    ros::spin();

    ROS_INFO_STREAM_NAMED("hardware_interface","Shutting down.");

    return 0;
}
