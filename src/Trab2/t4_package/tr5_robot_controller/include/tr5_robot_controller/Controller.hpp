/***************************** Made by Duarte Gonçalves and André Lourenço *********************************/

#ifndef _TR5_CONTROLLER_HPP__
#define _TR5_CONTROLLER_HPP__

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tr5_robot_controller/KinematicMode.h>
#include <tr5_kinematics/DoForwardKinematic.h>
#include <tr5_kinematics/DoInverseKinematic.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>



typedef visualization_msgs::InteractiveMarkerFeedbackConstPtr IMarkerConstPtr;


namespace TR5
{


    class Controller
    {
        public:

            Controller ();
            virtual ~Controller (){};

            void run();

        private:

            bool                                          m_inverse_mode;
            ros::NodeHandle                               m_nd;
            ros::Publisher                                m_pub_goal_jts; ///goal_joints
            ros::Publisher                                m_pub_goal_ps; ///goal_pose
            ros::ServiceServer                            m_pub_kinematicsMode; ///kin_mode
            ros::Subscriber                               m_sub_gui_jt; ///gui_joints
            ros::Subscriber                               m_sub_input_pose_feedback; ///input_pose/feedback
            ros::ServiceClient                            m_do_fk;
            ros::ServiceClient                            m_do_ik;
            visualization_msgs::Marker                    m_marker;
            visualization_msgs::InteractiveMarker         m_imarker;
            interactive_markers::InteractiveMarkerServer  m_marker_server;


            void create_interactive_marker();

            void gui_joints_clbk ( const sensor_msgs::JointState::ConstPtr& msg);
            void input_pose_clbk ( const IMarkerConstPtr&              feedback);
            bool changeKinMode(tr5_robot_controller::KinematicMode::Request&   req, tr5_robot_controller::KinematicMode::Response&  res);
    };
}

#endif /* _TR5_CONTROLLER_HPP__ */
