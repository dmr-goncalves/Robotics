/***************************** Made by Duarte Gonçalves and André Lourenço *********************************/
#ifndef _OBSTACLE_DETECTOR_HPP__
#define _OBSTACLE_DETECTOR_HPP__

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nxt_msgs/Range.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

class ObstacleDetector
{
public:

    ObstacleDetector();
    virtual ~ObstacleDetector(){};

    void run();

private:
    ros::NodeHandle                               nh4;
    ros::Publisher                                m_pub_map; // /nxt/map
    ros::Publisher                                m_pub_Range; // / /nxt/range
    ros::Subscriber                               m_sub_ultrasonicSensor; // /nxt/ultrasonic_sensor
    ros::Subscriber                               m_sub_odom; // /nxt/odom
    float                                         range;
    float                                         range_min;
    float                                         range_max;
    float                                         spread_angle;
    nav_msgs::OccupancyGrid                       occupGrid;
    double                                        nxtAngle; //To know what's the nxt orientation
    nav_msgs::Odometry                            odomNXT; //Save odometry values
    float                                         mapResolution;
    int                                           mapWidth;
    int                                           mapHeight;
    double                                        mapOriginPosX;
    double                                        mapOriginPosY;
    double                                        mapOriginPosZ;
    double                                        mapOriginOrX;
    double                                        mapOriginOrY;
    double                                        mapOriginOrZ;
    double                                        mapOriginOrW;
    int                                           gridX;
    int                                           gridY;
    int                                           gridXW;
    int                                           gridYW;
    double                                        xI;
    double                                        yI;
    double                                        xF;
    double                                        yF;
    std_msgs::Float64                             range2Send;
    void ultrasonicSensorClbk(const nxt_msgs::Range msg);
    void OdomClbk (const nav_msgs::Odometry msg);
};


#endif /* _OBSTACLE_DETECTOR_HPP__ */
