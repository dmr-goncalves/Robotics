/***************************** Made by Duarte Gonçalves and André Lourenço *********************************/
#ifndef _NXT_MAZE_SOLVER_HPP__
#define _NXT_MAZE_SOLVER_HPP__

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>


class MazeSolver
{
public:

    MazeSolver();
    virtual ~MazeSolver(){};

    void run();

private:

    ros::NodeHandle                               nh0;
    ros::Timer                                    controllerTimer;
    ros::Publisher                                m_pub_moveNXT; // /nxt/moveNXT
    ros::Subscriber                               m_sub_MagneitcFound; // /nxt/magnetic_found
    ros::Subscriber                               m_sub_intensitySensor; // /nxt/line_in_sight
    ros::Subscriber                               m_sub_odom;
    ros::Subscriber                               m_sub_Go;
    ros::Subscriber                               m_sub_MagneitcPose; // /nxt/magnetic_pose
    ros::Subscriber                               m_sub_map; // /nxt/map
    ros::Subscriber                               m_sub_Range; // /nxt/range
    std_msgs::Bool                                lineDetected;//Detected line or not
    std_msgs::Bool                                magneticFound;//Found magnetic dipole or not
    std_msgs::Bool                                go;//To see if it is calibrated
    geometry_msgs::Twist                          moveNXT;// Message to the motors
    geometry_msgs::Pose                           magneticPose;//Pose from the magnetic dipole
    nav_msgs::OccupancyGrid                       occupancyGrid; //OccupancyGrid with the obstacles
    double                                        yaw; //Saves actual yaw
    double                                        yawLostBlack; //Saves the yaw from when he lost the line after came back
    nav_msgs::Odometry                            odomNXT; //Save odometry values
    int                                           cntr; // Wait some time in the white area
    int                                           cntr2; //Turn back with the same speed and same duration
    long                                          cntr3; //Change Zone
    float                                         range;
    bool                                          turned90Degrees;
    bool                                          lineObstacleAux;
    bool                                          lostBlack4FirstTime; //To save the yaw from where the nxt lost the line
    bool                                          objectFoundFirstTime; //To save the yaw from where we found the object
    bool                                          firsTime; //To start the timer only once
    bool                                          searchedLeft;//Searched left and we can go search right
    bool                                          searchedRight;//Searched right and we can go search left
    bool                                          searchedLeftRight1; //To know if left was searched
    bool                                          searchedLeftRight2; //To know if right was searched
    bool                                          zone1; //To know which zone the nxt is
    int                                           magneticAlreadyFound; //To see if we found the dipole
    bool                                          turnedRight;
    bool                                          goneFront;
    bool                                          turnedLeft;
    bool                                          goneFrontAgain;
    bool                                          turnedLeftAgain;
    int                                           obstacleCntr;
    int                                           obstacleCntrAux;
    int                                           lineObstacleCntr;
    int                                           lineObstacleCntrAux;
    int                                           lineObstacleCntr2;
    int                                           lineObstacleCntrAux2;
    int                                           lineObstacleCntr3;
    int                                           lineObstacleCntrAux3;
    int                                           lineObstacleCntr4;
    int                                           lineObstacleCntrAux4;
    int                                           lineObstacleCntr5;
    int                                           lineObstacleCntrAux5;
    int                                           lineObstacleCntr6;
    int                                           lineObstacleCntrAux6;
    int                                           rightCntr;
    int                                           leftCntr;
    void commandVelClbk ( const geometry_msgs::Twist::ConstPtr& msg); //Callback from the rqt_steering
    void IntensitySensorClbk ( const std_msgs::Bool msg);//Callback from the intesity sensor to see if line is detected or not
    void intensityClbk(const std_msgs::Float64 msgs);
    void goClbk ( const std_msgs::Bool msg); //Callback from the intesity sensor to see if it is calibrated
    void magneticDipoleFinderClbk(const std_msgs::Bool msg); //To know the status of the search for the magnetic dipole
    void magneticPoseClbk(const geometry_msgs::Pose msg); //To save the position from the magnetic dipole
    void OdomClbk ( const nav_msgs::Odometry msg); //Callback from the wheels with the odometry
    void MapClbck(const nav_msgs::OccupancyGrid msg); //Callback with the map from the obstacle detector
    void rangeClbck(const std_msgs::Float64 msg); //Callback with the range from the obstacle detector
    void Controller(); //Controller called by a timer
    void timerClbk(const ros::TimerEvent& event);
    void goRight(); //Turn right
    void goLeft(); //Turn left
    void goFront(); //Only forward
    void goBack(); //Only backward
    void Stop(); //Stops the robot
    void turnLeftAroundHimself(); //Turn robot around himself to the left
    void turnRightAroundHimself(); //Turn robot around himself to the right
    double toDegrees(double radian); //Convert radians to degrees
};


#endif /* _NXT_MAZE_SOLVER_HPP__ */
