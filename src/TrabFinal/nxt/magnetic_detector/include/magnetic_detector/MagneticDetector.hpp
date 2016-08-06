/***************************** Made by Duarte Gonçalves and André Lourenço *********************************/
#ifndef _MAGNETIC_DETECTOR_HPP__
#define _MAGNETIC_DETECTOR_HPP__

#include <ros/ros.h>
#include <nxt_msgs/MagneticField.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

class MagneticDetector
    {
        public:

            MagneticDetector();
            virtual ~MagneticDetector(){};

            void run();

        private:

            ros::NodeHandle                               nh3;
            ros::Publisher                                m_pub_magneticPose; // /nxt/magnetic_pose
            ros::Publisher                                m_pub_magneticFound; // /nxt/magnetic_found
            ros::Publisher                                m_pub_magneticMarker; //nxt/magneticMarker
            ros::Subscriber                               m_sub_magneticSensor; // ​/nxt/magnetic_sensor
            ros::Subscriber                               m_sub_odom; // ​/nxt/odom
            geometry_msgs::Pose                           nxtPose;
            int                                           magneticThreshold; //Magnetic value threshold
            void MagneticSensorClbk ( const nxt_msgs::MagneticField msg);
            void OdomClbk (const nav_msgs::Odometry msg);
    };


#endif /* _MAGNETIC_DETECTOR_HPP__ */
