/***************************** Made by Duarte Gonçalves and André Lourenço  *********************************/

#include "nxt_maze_solver/MazeSolver.hpp"

//Central node
MazeSolver::MazeSolver(){
	lineDetected.data = false;
	magneticFound.data = false;
	m_sub_MagneitcFound = nh0.subscribe("/nxt/magnetic_found", 1, &MazeSolver::magneticDipoleFinderClbk, this);
	m_sub_MagneitcPose = nh0.subscribe("/nxt/magnetic_pose", 1, &MazeSolver::magneticPoseClbk, this);
	m_sub_intensitySensor = nh0.subscribe("/nxt/line_in_sight", 1, &MazeSolver::IntensitySensorClbk, this);
	m_sub_map = nh0.subscribe("/nxt/map", 1, &MazeSolver::MapClbck, this);
	m_sub_Go = nh0.subscribe("/nxt/goNxt", 1, &MazeSolver::goClbk, this);
	m_sub_odom = nh0.subscribe("/nxt/odom", 1, &MazeSolver::OdomClbk, this);
	m_sub_Range = nh0.subscribe("/nxt/range",1, &MazeSolver::rangeClbck, this);
	m_pub_moveNXT = nh0.advertise<geometry_msgs::Twist>("/nxt/moveNXT", 1);
	controllerTimer = nh0.createTimer(ros::Duration(1/50), &MazeSolver::timerClbk, this, false, false);
	firsTime = true;
	cntr = 0;
	cntr2 = 0;
	lostBlack4FirstTime = true;
	objectFoundFirstTime = true;
	searchedLeft = false;
	searchedRight = false;
	searchedLeftRight1 = false;
	searchedLeftRight2 = false;
	zone1 = true;
	magneticAlreadyFound = 0;
	turned90Degrees = false;
	lineObstacleAux = false;
	obstacleCntr = 0;
	obstacleCntrAux = 0;
	rightCntr = 0;
	leftCntr = 0;
	turnedRight = false;
	turnedLeft = false;
	goneFront = false;
}

//Populate the geometry_msgs::Twist object with the values to move right
void MazeSolver::goRight(){
	moveNXT.linear.x = 0.08;
	moveNXT.angular.z = 0.025;
}

//Populate the geometry_msgs::Twist object with the values to move left
void MazeSolver::goLeft(){
	moveNXT.linear.x = 0.08;
	moveNXT.angular.z = -0.025;
}

//Populate the geometry_msgs::Twist object with the values to move forward
void MazeSolver::goFront(){
	moveNXT.linear.x = 0.1;
	moveNXT.angular.z = 0.0;
}

//Populate the geometry_msgs::Twist object with the values to move backwards
void MazeSolver::goBack(){
	moveNXT.linear.x = 0.08;
	moveNXT.angular.z = -0.025;
}

//Populate the geometry_msgs::Twist object with the values to stop
void MazeSolver::Stop(){
	moveNXT.linear.x = 0.0;
	moveNXT.angular.z = 0.0;
}

//Populate the geometry_msgs::Twist object with the values to move around himself to the right
void MazeSolver::turnLeftAroundHimself(){
	moveNXT.linear.x = 0.0;
	moveNXT.angular.z = -0.1;
}

//Populate the geometry_msgs::Twist object with the values to move around himself to the left
void MazeSolver::turnRightAroundHimself(){
	moveNXT.linear.x = 0.0;
	moveNXT.angular.z = 0.1;
}

void MazeSolver::run()
{
	ros::spin();
}

void MazeSolver::Controller(){
	if(go.data){//Finished calibrating
		if(zone1){//We are in the first zone
			if(range > 0.22 && !lineObstacleAux){
				if(lineDetected.data){ //Found line
					goLeft();
					cntr = 0;
					cntr2 = 0;
					cntr3 = 0;
					lostBlack4FirstTime = true;
					searchedRight = false;
					searchedLeft = false;
					searchedLeftRight1 = false;
					searchedLeftRight2 = false;
					lostBlack4FirstTime = 0.0;
					turnedRight = false;
					turnedLeft = false;
					turnedLeftAgain = false;
					goneFront = false;
					goneFrontAgain = false;
				}else{//Lost line
					if(cntr3 < 100000){//If we have searched for too long we are in the second zone
						if(cntr > 50000){ //Lost line for too much time so go search
							if(cntr2 > 50000){//Came back to orientation of the last line
								Stop();

								if(lostBlack4FirstTime){//Save the yaw from where we lost the line
									yawLostBlack = yaw;
									lostBlack4FirstTime = false;
								}

								if((yaw - yawLostBlack) > 93){//Search Right
									searchedRight = true;
									searchedLeft = false;
								}else if ((yaw - yawLostBlack) < -93){ //Search Left
									searchedLeft = true;
									searchedRight = false;
								}

								if ((yaw - yawLostBlack) > -3.0 && (yaw - yawLostBlack) < 3.0 && searchedLeftRight1 && searchedLeftRight2){//We already searchedboth sides so we go forward
									searchedRight = false;
									searchedLeft = false;
									Stop();
									goFront();
								}else if(searchedRight){ //Search Right
									//Stop();
									turnRightAroundHimself();
									searchedLeftRight1 = true;
								}else if(searchedLeft){ //Search Left
									//Stop();
									turnLeftAroundHimself();
									searchedLeftRight2 = true;
								}else{
									turnLeftAroundHimself();
								}
							}else{ //Lost line for too much time so we come back to the direction of the last line

								Stop();
								goBack();
								cntr2++;
								cntr3++;
							}
						}else{//Lost line
							goRight();
							cntr++;
							cntr3++;
						}
					}else{//Changed zone
						zone1 = false;
					}
				}
				m_pub_moveNXT.publish(moveNXT);//Send the values to motor_controller node
			}else{//Found obstacle so we search for the line in 90 degrees

				if(!lineObstacleAux){//Stop going forward
					lineObstacleAux = true;
				}

				if(lineDetected.data){ //Found line
					lineObstacleAux = false;
					cntr = 0;
					cntr2 = 0;
					lostBlack4FirstTime = true;
					searchedRight = false;
					searchedLeft = false;
					searchedLeftRight1 = false;
					searchedLeftRight2 = false;
					lostBlack4FirstTime = 0.0;
					turnedRight = false;
					turnedLeft = false;
					turnedLeftAgain = false;
					goneFront = false;
					goneFrontAgain = false;
				}else{//Lost line
					if(cntr > 50000){ //Lost line for too much time so go search
						if(cntr2 > 50000){//Came back to orientation of the last line
							Stop();

							if(lostBlack4FirstTime){//Save the yaw from where we lost the line
								yawLostBlack = yaw;
								lostBlack4FirstTime = false;
								lineObstacleAux = lineObstacleCntr;
							}

							if((yaw - yawLostBlack) > 93){//Search Right
								searchedRight = true;
								searchedLeft = false;
							}else if ((yaw - yawLostBlack) < -93){ //Search Left
								searchedLeft = true;
								searchedRight = false;
							}

							if(searchedLeftRight1 && searchedLeftRight2 && !searchedRight && !searchedLeft){
								if(lineObstacleCntr - lineObstacleCntrAux > 8){//Turned 90 degrees to the right
									if(!turnedRight){
										turnedRight = true;
										lineObstacleCntrAux2 = lineObstacleCntr2;
									}
									if( lineObstacleCntr2 - lineObstacleCntrAux2 > 8){ //Turn Left
										if(!goneFront){
											goneFront = true;
											lineObstacleCntrAux3 = lineObstacleCntr3;
										}
										if(lineObstacleCntr3 - lineObstacleCntrAux3 > 8){
											if(!turnedLeft){
												turnedLeft = true;
												lineObstacleCntrAux4 = lineObstacleCntr4;
											}
											if(lineObstacleCntr4 - lineObstacleCntrAux4 > 8){
												if(!goneFrontAgain){
													goneFrontAgain = true;
													lineObstacleCntrAux5 = lineObstacleCntr5;
												}
												if(lineObstacleCntr5 - lineObstacleCntrAux5 > 8){
													if(!turnedLeftAgain){
														turnedLeftAgain = true;
														lineObstacleCntrAux6 = lineObstacleCntr6;
													}
													if(lineObstacleCntr6 - lineObstacleCntrAux6 > 8){
														lineObstacleAux = false;
													}else{//Keep going forward
														goFront();
													}
												}else{//Keep turning left
													turnLeftAroundHimself();
												}
											}else{//Keep going forward
												goFront();
											}
										}else{//Keep turning left
											turnLeftAroundHimself();
										}
									}else{//Keep going forward
										goFront();
									}
								}else{//Continue turning right
									turnRightAroundHimself();
								}
							}else{
								if ((yaw - yawLostBlack) > -3.0 && (yaw - yawLostBlack) < 3.0 && searchedLeftRight1 && searchedLeftRight2){//We already searchedboth sides so we go forward
									searchedRight = false;
									searchedLeft = false;
									Stop();
								}else if(searchedRight){ //Search Right
									//Stop();
									turnRightAroundHimself();
									searchedLeftRight1 = true;
								}else if(searchedLeft){ //Search Left
									//Stop();
									turnLeftAroundHimself();
									searchedLeftRight2 = true;
								}else{
									turnLeftAroundHimself();
								}
							}
						}
						else{ //Lost line for too much time so we come back to the direction of the last line
							Stop();
							goBack();
							cntr2++;
							cntr3++;
						}
					}else{//Lost line
						goRight();
						cntr++;
						cntr3++;
					}
				}
			}
		}else{//We are in zone two
			if(magneticAlreadyFound != 0){ //Search for the exit
				if(lineDetected.data){//Found the exit
					Stop();
					ROS_INFO("Finished Solving the Maze");
					m_pub_moveNXT.publish(moveNXT);
				}
			}else{ //Search for the dipole
				if(range > 0.22 && !turned90Degrees){ // Didn't find obstacle. turned90Degrees is a control variable so we know if we already turned 90 degrees from the last obstacle
				objectFoundFirstTime = true;
				goFront();

				if(lineDetected.data){
					ROS_WARN("Found the line but not the dipole");
					Stop();
					turned90Degrees = true;
				}
			}else{//Detected obstacle

				if(!turned90Degrees){//Stop going forward
					turned90Degrees = true;
				}

				if(objectFoundFirstTime){ //Save the counter from where we found the obstacle
					objectFoundFirstTime = false;
					obstacleCntrAux = obstacleCntr;
				}

				if(rightCntr < 3){
					if(obstacleCntr - obstacleCntrAux > 8){//Turned 90 degrees so we can go forward
						goFront();
						turned90Degrees = false;
						rightCntr++;
					}else{//Continue turning
						turnRightAroundHimself();
					}
					leftCntr = 0;
				}else{//Already turned right 3 times
					if(leftCntr < 2){
						if(obstacleCntr - obstacleCntrAux > 8){
							goFront();
							turned90Degrees = false;
							leftCntr++;
						}else{
							turnLeftAroundHimself();
						}
					}else{
						rightCntr = 0;
					}
				}
			}
			m_pub_moveNXT.publish(moveNXT);
		}
	}
  }
}

//Convert radians to degrees
double MazeSolver::toDegrees(double radian){
	return radian * 180 / M_PI;
}

//Save Odometry values
void MazeSolver::OdomClbk(const nav_msgs::Odometry msg){
	yaw = toDegrees(tf::getYaw(msg.pose.pose.orientation));
	odomNXT = msg;
	obstacleCntr++;
	lineObstacleCntr++;
	lineObstacleCntr2++;
	lineObstacleCntr3++;
	lineObstacleCntr4++;
	lineObstacleCntr5++;
	lineObstacleCntr6++;
}

//Sees the timer event
void MazeSolver::timerClbk( const ros::TimerEvent& event){
	Controller();
}

//Receives the values of the light sensor and starts the timer
void MazeSolver::IntensitySensorClbk ( const std_msgs::Bool msg){
	lineDetected = msg;
	if(firsTime){
		controllerTimer.start();
		firsTime = false;
	}

}

//Receives the aknowlegment from the calibration
void MazeSolver::goClbk ( const std_msgs::Bool msg){
	go = msg;
}

//Receives the magnetic found pose and aknowlegment
void MazeSolver::magneticDipoleFinderClbk(const std_msgs::Bool msg){
	magneticFound = msg;
	if(magneticFound.data){
		magneticAlreadyFound++;
	}
}

//Receives the map
void MazeSolver::MapClbck(const nav_msgs::OccupancyGrid  msg){
	occupancyGrid = msg;
}

//Receives the range
void MazeSolver::rangeClbck(const std_msgs::Float64 msg){
	range = msg.data;
}

//Save the pose from the magnetic dipole
void MazeSolver::magneticPoseClbk(const geometry_msgs::Pose msg){
	magneticPose = msg;
}
