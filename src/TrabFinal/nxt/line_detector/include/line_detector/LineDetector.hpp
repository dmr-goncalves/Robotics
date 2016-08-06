/***************************** Made by Duarte Gonçalves and André Lourenço *********************************/
#ifndef _LINE_DETECTOR_HPP__
#define _LINE_DETECTOR_HPP__

#include <ros/ros.h>
#include <nxt_msgs/Light.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

class LineDetector
    {
        public:

            LineDetector();
            virtual ~LineDetector(){};

            void run();

        private:

            ros::NodeHandle                               nh2;
            ros::Publisher                                m_pub_lineInSight; // /nxt/line_in_sight
            ros::Publisher                                m_pub_Go;
            ros::Publisher                                m_pub_Intensity;
            ros::Subscriber                               m_sub_intensitySensor; // /nxt/intensity_sensor
            std_msgs::Bool                                blackLineDetected;
            bool                                          isCalibrated;
            double                                        msgIntensity;
            std_msgs::Float64                             intens;
            double                                        intensityCalibrated;
            void IntensitySensorClbk( const nxt_msgs::Light msg);
    };
#endif /* _LINE_DETECTOR_HPP__ */
