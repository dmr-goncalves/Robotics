/***************************** Made by Duarte Gonçalves and André Lourenço  *********************************/

#include "magnetic_detector/MagneticDetector.hpp"


MagneticDetector::MagneticDetector(){
   m_sub_magneticSensor = nh3.subscribe( "/nxt/magnetic_sensor", 1, &MagneticDetector::MagneticSensorClbk, this );
   m_sub_odom = nh3.subscribe("/nxt/odom", 1, &MagneticDetector::OdomClbk, this);
   m_pub_magneticPose = nh3.advertise<geometry_msgs::Pose>("/nxt/magnetic_pose", 1);
   m_pub_magneticFound = nh3.advertise<std_msgs::Bool>("/nxt/magnetic_found", 1);
   m_pub_magneticMarker = nh3.advertise<visualization_msgs::Marker>("/nxt/magneticMarker", 1);
   magneticThreshold = 20;
}

void MagneticDetector::run(){
   ros::spin();

}

//Save the Odometry values
void MagneticDetector::OdomClbk ( const nav_msgs::Odometry msg){
   nxtPose = msg.pose.pose;
}

//Receives the magnetic sensor value and decide if the dipole was detected. If so we send a marker with the position to rviz and a pose to the central node
void MagneticDetector::MagneticSensorClbk ( const nxt_msgs::MagneticField msg){
   std_msgs::Bool found;
   if(msg.value > magneticThreshold){

      visualization_msgs::Marker                    m_marker;

      // visual marker object for RVIZ
      m_marker.ns              = "magnetic found marker";
      m_marker.id              = 0;
      m_marker.header.stamp    = ros::Time();
      m_marker.header.frame_id = "odom";
      m_marker.color.a         = 0.5;
      m_marker.color.g         = 1.0;
      m_marker.color.b         = m_marker.color.r = 0.0;
      m_marker.scale.x         = m_marker.scale.y = m_marker.scale.z = 0.10;
      m_marker.action          = visualization_msgs::Marker::ADD;
      m_marker.type            = visualization_msgs::Marker::SPHERE;
      m_marker.lifetime        = ros::Duration();

      //
      // update RVIZ marker's pose from forward kinematics
      //

      m_marker.pose.position.x = nxtPose.position.x;
      m_marker.pose.position.y = nxtPose.position.y;
      m_marker.pose.position.z = 0.0;
      m_marker.pose.orientation.x = 0.0;
      m_marker.pose.orientation.y = 0.0;
      m_marker.pose.orientation.z = 0.0;
      m_marker.pose.orientation.w = 0.0;


      m_pub_magneticMarker.publish(m_marker); //Put a marker on the dipole position

      found.data = true;
      m_pub_magneticFound.publish(found); //Acknowledge the detection of the dipole
      m_pub_magneticPose.publish(nxtPose); //Send the pose from where we found the dipole
   }else{
      found.data = false;
      m_pub_magneticFound.publish(found);
   }
}
