/***************************** Made by Duarte Gonçalves and André Lourenço  *********************************/

#include "obstacle_detector/ObstacleDetector.hpp"


ObstacleDetector::ObstacleDetector(){

	m_sub_ultrasonicSensor = nh4.subscribe( "/nxt/ultrasonic_sensor", 1, &ObstacleDetector::ultrasonicSensorClbk, this );
	m_sub_odom = nh4.subscribe( "/nxt/odom", 1, &ObstacleDetector::OdomClbk, this );
	m_pub_map = nh4.advertise<nav_msgs::OccupancyGrid>("/nxt/map", 1);
	m_pub_Range = nh4.advertise<std_msgs::Float64>("/nxt/range", 1);
	//Map dimensions, position and orientation
	mapResolution = 0.01;
	mapWidth = 500;
	mapHeight = 500;
	mapOriginPosX = 0.0;
	mapOriginPosY = 0.0;
	mapOriginPosZ = 0.0;
	mapOriginOrX = 0.0;
	mapOriginOrY = 0.0;
	mapOriginOrZ = 0.0;
	mapOriginOrW = 0.0;
}

void ObstacleDetector::run(){
	//Populate the OccupancyGrid object with the dimensions, position and orientation
	occupGrid.info.resolution = mapResolution;
	occupGrid.info.width = mapWidth;
	occupGrid.info.height = mapHeight;
	occupGrid.info.origin.position.x = mapOriginPosX;
	occupGrid.info.origin.position.y = mapOriginPosY;
	occupGrid.info.origin.position.z = mapOriginPosZ;
	occupGrid.info.origin.orientation.x = mapOriginOrX;
	occupGrid.info.origin.orientation.y = mapOriginOrY;
	occupGrid.info.origin.orientation.z = mapOriginOrZ;
	occupGrid.info.origin.orientation.w = mapOriginOrW;

	for(unsigned int i=0; i < mapWidth * mapHeight; i++){ //First time we run so all the locations are unknown
		occupGrid.data.push_back(-1);
	}
	ros::spin();
}

//Receives the ultrasonic sensor values and builds the map
void ObstacleDetector::ultrasonicSensorClbk(const nxt_msgs::Range msg){
	range = msg.range;
	range_min = msg.range_min;
	range_max = msg.range_max;
	spread_angle = msg.spread_angle;

	xI = odomNXT.pose.pose.position.x * 100; //Actual Position Coordenates
	yI = odomNXT.pose.pose.position.y * 100;

	range2Send.data = range;
	m_pub_Range.publish(range2Send);

	if(range > range_min && range < range_max){//Potential obstacle detected
		if(range > 2){ //Limite the range so we don't have false positives
		range = 2;
	}

	if(range < 0.25){//We don't erase things if the range is to small
	range = 0.25 - range;
}

for(float z = -spread_angle/2.0; z < spread_angle/2.0; z += 0.01){ //Paint the spread_angle view

	xF = xI + range * cos(nxtAngle + z) * 100; //Actual Position Plus Range Coordenates
	yF = yI + range * sin(nxtAngle + z) * 100;

	gridX = (int) (xF + mapWidth / 2); //Coordenates from the grid of the xF
	gridY = (int) (yF + mapHeight / 2); //Coordenates from the grid of the yF
	int val = (int)(((range_min * 100 - range_max * 100) / 100) * range * 100 + range_max * 100);//Change the probability value dynamically according to the range seen

	if(val > 100){//Maximum probability
		val = 100;
	}

	if(val < 0){//Minimum probability
		val = 0;
	}

	if(range > 0.25 && range < 0.75){//In this value range we are sure that we have detected an object
		val = 100;
	}

	occupGrid.data[gridX + gridY * mapWidth] = val;//Populate the OccupancyGrid data vector

	for(float f = 0.01; f < range; f += 0.01){ //Paint white cone until we reach our range

		xF = xI + f * cos(nxtAngle + z) * 100; //Actual Position Plus Range Coordenates
		yF = yI + f * sin(nxtAngle + z) * 100;

		gridXW = (int) (xF + mapWidth / 2); //Coordenates from the grid of the xF
		gridYW = (int) (yF + mapHeight / 2); //Coordenates from the grid of the yF

		if((gridXW != gridX) || (gridY != gridYW)){//We have seen black in this position but now we don't so we probably were mistaken and we clean that position
		occupGrid.data[gridXW + gridYW * mapWidth] = 0;
	}
}
}

m_pub_map.publish(occupGrid);//Send the OccupancyGrid to the rviz and the central node
}
}

//Receives the Odometry values and saves them
void ObstacleDetector::OdomClbk(const nav_msgs::Odometry msg){
	odomNXT = msg;
	nxtAngle = tf::getYaw(odomNXT.pose.pose.orientation);
}
