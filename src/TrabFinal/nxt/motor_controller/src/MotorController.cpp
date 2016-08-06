/***************************** Made by Duarte Gonçalves and André Lourenço  *********************************/

#include "motor_controller/MotorController.hpp"


MotorController::MotorController(){
	wheelRadius = 0.028;
	m_sub_moveNXT = nh1.subscribe("/nxt/moveNXT", 1, &MotorController::moveNXTClbk, this);
	m_pub_jtCommand = nh1.advertise<sensor_msgs::JointState>("/nxt/joint_command", 1);
}

void MotorController::run()
{
	ros::spin();
}

//Convert the velocity into effort
float MotorController::convertVel2Effort(float velocity){
	return (5.882352941 * velocity) / wheelRadius ;
}

//Sends the effort values to the motors
void MotorController::moveNXTClbk ( const geometry_msgs::Twist::ConstPtr& msg){
	linearVelocity = msg->linear.x;
	angularVelocity = msg->angular.z;
	linearEffort = convertVel2Effort(linearVelocity);
	angularEffort = angularVelocity / wheelRadius;

	if(linearVelocity == 0 && angularVelocity == 0){ //Stop
		sensor_msgs::JointState jtState;
		jtState.name.push_back("nxt/right_motor_joint");
		jtState.name.push_back("nxt/left_motor_joint");
		jtState.position.push_back(0.0);
		jtState.position.push_back(0.0);
		jtState.velocity.push_back(0.0);
		jtState.velocity.push_back(0.0);
		jtState.effort.push_back(0);
		jtState.effort.push_back(0);
		jtState.header.stamp = ros::Time::now();
		m_pub_jtCommand.publish(jtState);//Send the joint states populated with the efforts and name to the motors
	}else if(linearVelocity != 0 && angularVelocity == 0){//Just walk forward or backward
		sensor_msgs::JointState jtState;
		jtState.name.push_back("nxt/right_motor_joint");
		jtState.name.push_back("nxt/left_motor_joint");
		jtState.position.push_back(0.0);
		jtState.position.push_back(0.0);
		jtState.velocity.push_back(0.0);
		jtState.velocity.push_back(0.0);
		jtState.effort.push_back(linearEffort);
		jtState.effort.push_back(linearEffort);
		jtState.header.stamp = ros::Time::now();
		m_pub_jtCommand.publish(jtState);//Send the joint states populated with the efforts and name to the motors
	}
	else if(linearVelocity == 0 && angularVelocity != 0){//Just turn around himself
		sensor_msgs::JointState jtState;
		jtState.name.push_back("nxt/right_motor_joint");
		jtState.name.push_back("nxt/left_motor_joint");
		jtState.position.push_back(0.0);
		jtState.position.push_back(0.0);
		jtState.velocity.push_back(0.0);
		jtState.velocity.push_back(0.0);

		if(angularVelocity < 0){
			jtState.effort.push_back(-angularEffort);
			jtState.effort.push_back(angularEffort);
		}else{
			jtState.effort.push_back(-angularEffort);
			jtState.effort.push_back(angularEffort);
		}

		jtState.header.stamp = ros::Time::now();
		m_pub_jtCommand.publish(jtState);//Send the joint states populated with the efforts and name to the motors
	}else if(linearVelocity != 0  && angularVelocity != 0){//Walk forward but also to the sides
		sensor_msgs::JointState jtState;
		jtState.name.push_back("nxt/right_motor_joint");
		jtState.name.push_back("nxt/left_motor_joint");
		jtState.position.push_back(0.0);
		jtState.position.push_back(0.0);
		jtState.velocity.push_back(0.0);
		jtState.velocity.push_back(0.0);

		if(angularVelocity < 0){
			jtState.effort.push_back(0.7 * linearEffort);
			jtState.effort.push_back(0.3 * angularEffort);
		}else{
			jtState.effort.push_back(-0.3 * angularEffort);
			jtState.effort.push_back(0.7 * linearEffort);
		}

		jtState.header.stamp = ros::Time::now();
		m_pub_jtCommand.publish(jtState);//Send the joint states populated with the efforts and name to the motors
	}
}
