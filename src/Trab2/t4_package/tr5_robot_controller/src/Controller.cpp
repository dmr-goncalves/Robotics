/***************************** Made by Duarte Gonçalves and André Lourenço *********************************/

#include "tr5_robot_controller/Controller.hpp"

using namespace TR5;


Controller::Controller()

: m_nd("~"), m_marker_server("input_pose"), m_inverse_mode(false)

{
    std::string tp_sel_kin, tp_gui_jt, tp_goal_jt, tp_goal_ps, tp_inpt_ps;
    std::string tp_inv_k, tp_for_k;

    // Load private parameters from Parameter Server
    m_nd.param<std::string>("select_kin_topic",    tp_sel_kin, "/kin_mode"   );
    m_nd.param<std::string>("gui_joints_topic",    tp_gui_jt,  "/gui_joints" );
    m_nd.param<std::string>("goal_joints_topic",   tp_goal_jt, "/goal_joints");
    m_nd.param<std::string>("goal_pose_topic",     tp_goal_ps, "/goal_pose"  );
    m_nd.param<std::string>("inverse_kinet_topic", tp_inv_k,   "/do_ik"      );
    m_nd.param<std::string>("forward_kinet_topic", tp_for_k,   "/do_fk"      );

    if ( !m_nd.hasParam("select_kin_topic") )
    ROS_WARN("No param 'select_kin_topic'\tDefault: %s",  tp_sel_kin.c_str());
    if ( !m_nd.hasParam("gui_joints_topic") )
    ROS_WARN("No param 'gui_joints_topic'\tDefault: %s",  tp_gui_jt.c_str());
    if ( !m_nd.hasParam("goal_joints_topic") )
    ROS_WARN("No param 'goal_joints_topic'\tDefault: %s", tp_goal_jt.c_str());
    if ( !m_nd.hasParam("goal_pose_topic") )
    ROS_WARN("No param 'goal_pose_topic'\tDefault: %s",   tp_goal_ps.c_str());
    if ( !m_nd.hasParam("forward_kinet_topic") )
    ROS_WARN("No param 'forward_kinet_topic'\tDefault: %s", tp_for_k.c_str());
    if ( !m_nd.hasParam("inverse_kinet_topic") )
    ROS_WARN("No param 'inverse_kinet_topic'\tDefault: %s", tp_inv_k.c_str());

    // visual marker object for RVIZ
    m_marker.ns              = "forward kinematics marker";
    m_marker.id              = 0;
    m_marker.header.stamp    = ros::Time();
    m_marker.header.frame_id = "world";
    m_marker.color.a         = 0.5;
    m_marker.color.r         = 1.0;
    m_marker.color.b         = m_marker.color.g = 0.0;
    m_marker.scale.x         = m_marker.scale.y = m_marker.scale.z = 0.05;
    m_marker.action          = visualization_msgs::Marker::ADD;
    m_marker.type            = visualization_msgs::Marker::SPHERE;
    m_marker.lifetime        = ros::Duration();


    // initialise RVIZ's interactive marker to use in inverse kinematics mode
    create_interactive_marker();

    //
    // initialise ROS service and message publishers/subscribers
    //
    //Subscribers
    m_sub_gui_jt = m_nd.subscribe( tp_gui_jt, 1, &Controller::gui_joints_clbk, this );
    m_sub_input_pose_feedback = m_nd.subscribe("/input_pose/feedback", 1, &Controller::input_pose_clbk, this );

    //Publishers
    m_pub_goal_jts = m_nd.advertise<sensor_msgs::JointState>(tp_goal_jt, 1);
    m_pub_goal_ps = m_nd.advertise<visualization_msgs::Marker>(tp_goal_ps, 1);

    //Service Publishers
    m_pub_kinematicsMode = m_nd.advertiseService(tp_sel_kin, &Controller::changeKinMode, this);

    //Service Subscribers
    m_do_fk = m_nd.serviceClient<tr5_kinematics::DoForwardKinematic>(tp_for_k);
    m_do_ik = m_nd.serviceClient<tr5_kinematics::DoInverseKinematic>(tp_inv_k);
}




void Controller::run()
{
    ros::spin();
}



bool Controller::changeKinMode(tr5_robot_controller::KinematicMode::Request&   req, tr5_robot_controller::KinematicMode::Response&  res)
{
    if(req.mode){ //Inverse kinematics
        ROS_INFO("Inverse Kinematics Mode");
        m_inverse_mode = true;
    }else if(!req.mode){ //Forward kinematics
        ROS_INFO("Forward Kinematics Mode");
        m_inverse_mode = false;
    }
    res.result = true;
}

void Controller::gui_joints_clbk ( const sensor_msgs::JointState::ConstPtr& msg )
{
    // ignore values from JointState GUI when in inverse kinematics mode
    if ( m_inverse_mode )
    return;

    //
    // create and fill the ROS service object with the joint information
    //

    tr5_kinematics::DoForwardKinematic srv;
    srv.request.fjState.name = msg->name;
    srv.request.fjState.position = msg->position;
    srv.request.fjState.velocity = msg->velocity;
    srv.request.fjState.effort = msg->effort;


    if (m_do_fk.call(srv))
    {
        //
        // update RVIZ marker's pose from forward kinematics
        //
        m_marker.pose.position.x = srv.response.fPose.position.x;
        m_marker.pose.position.y = srv.response.fPose.position.y;
        m_marker.pose.position.z = srv.response.fPose.position.z;

        m_pub_goal_ps.publish(m_marker);
    }
    else
    ROS_ERROR( "Failed to call service FORWARD KINEMATICS" );

    //
    //  publish joint values from JointState GUI to the TR5 driver node
    //
    m_pub_goal_jts.publish(msg);

}

void Controller::input_pose_clbk( const IMarkerConstPtr &feedback )
{
    // ignore feedback from interactive marker when in forward kinematics mode
    if ( !m_inverse_mode )
    return;

    tr5_kinematics::DoInverseKinematic srv;
    srv.request.iPose.position.x = feedback->pose.position.x + 0.55;
    srv.request.iPose.position.y = feedback->pose.position.y;
    srv.request.iPose.position.z = feedback->pose.position.z + 0.27;
    srv.request.iPose.orientation.x = feedback->pose.orientation.x;
    srv.request.iPose.orientation.y = feedback->pose.orientation.y;
    srv.request.iPose.orientation.z = feedback->pose.orientation.z;
    srv.request.iPose.orientation.w = feedback->pose.orientation.w;


    if (m_do_ik.call(srv))
    {
        //
        // publish goal joint values to the TR5 driver node
        //
        sensor_msgs::JointState jtState;
        jtState.name.push_back(srv.response.ijState.name.at(0));
        jtState.name.push_back(srv.response.ijState.name.at(1));
        jtState.name.push_back(srv.response.ijState.name.at(2));
        jtState.name.push_back(srv.response.ijState.name.at(3));
        jtState.name.push_back(srv.response.ijState.name.at(5));
        jtState.name.push_back(srv.response.ijState.name.at(4));
        jtState.name.push_back(srv.response.ijState.name.at(6));
        jtState.position.push_back(srv.response.ijState.position.at(0));
        jtState.position.push_back(srv.response.ijState.position.at(1));
        jtState.position.push_back(srv.response.ijState.position.at(2));
        jtState.position.push_back(srv.response.ijState.position.at(3));
        jtState.position.push_back(srv.response.ijState.position.at(4));
        jtState.position.push_back(srv.response.ijState.position.at(5));
        jtState.position.push_back(srv.response.ijState.position.at(6));
        jtState.header.stamp = srv.response.ijState.header.stamp;
        m_pub_goal_jts.publish(jtState);
    }
    else
    ROS_ERROR( "Failed to call service INVERSE KINEMATICS" );
}

void Controller::create_interactive_marker()
{
    // create an interactive marker
    m_imarker.header.frame_id = "world";
    m_imarker.header.stamp    = ros::Time::now();
    m_imarker.name            = "user_input_marker";
    m_imarker.description     = "Input Pose";

    // create a grey box marker
    visualization_msgs::Marker dot_marker;
    dot_marker.type = visualization_msgs::Marker::SPHERE;
    dot_marker.scale.x = dot_marker.scale.y = dot_marker.scale.z = 0.05;
    dot_marker.color.r = dot_marker.color.g = 0.0;
    dot_marker.color.b = 1.0;
    dot_marker.color.a = 0.5;
    dot_marker.pose.position.x = 0.55;
    dot_marker.pose.position.y = 0.0;
    dot_marker.pose.position.z = 0.27;

    // create a non-interactive control which contains the dot marker
    visualization_msgs::InteractiveMarkerControl dot_control;
    dot_control.always_visible = true;
    dot_control.markers.push_back( dot_marker );
    dot_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    dot_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;

    // add the control to the interactive marker
    m_imarker.controls.push_back( dot_control );

    m_marker_server.insert( m_imarker, boost::bind( &Controller::input_pose_clbk, this, _1 ) );

    m_marker_server.applyChanges();
    }
