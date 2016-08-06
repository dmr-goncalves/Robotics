/***************************** Made by Duarte Gonçalves and André Lourenço  *********************************/

#include "tr5_kinematics/Kinematics.hpp"

using namespace TR5;


Kinematics::Kinematics()
{
	m_pub_do_fk = nh.advertiseService("/do_fk", &Kinematics::do_fk, this);
	m_pub_do_ik = nh.advertiseService("/do_ik", &Kinematics::do_ik, this);
	liftJointHeight = 0.275;
	upperArmLenght = 0.200;
	forearmLenght = 0.130;
	endEffectorLenght = 0.123;
}

void Kinematics::run()
{
	ros::spin();
}

bool Kinematics::do_fk (tr5_kinematics::DoForwardKinematic::Request  &req, tr5_kinematics::DoForwardKinematic::Response &res )
{
	//
	//compute forward kinematics
	//
	float pan_joint_Angle = (float) req.fjState.position.at(0);
	float lift_joint_Angle = (float) req.fjState.position.at(1);
	float elbow_joint_Angle = (float) req.fjState.position.at(2);
	float wrist1_joint_Angle = (float) req.fjState.position.at(3);

	double xM = cos(pan_joint_Angle) * (upperArmLenght * cos(lift_joint_Angle) + forearmLenght * cos(lift_joint_Angle + elbow_joint_Angle) + endEffectorLenght * cos(lift_joint_Angle + elbow_joint_Angle + wrist1_joint_Angle));
	double yM = sin(pan_joint_Angle) * (upperArmLenght * cos(lift_joint_Angle) + forearmLenght * cos(lift_joint_Angle + elbow_joint_Angle) + endEffectorLenght * cos(lift_joint_Angle + elbow_joint_Angle + wrist1_joint_Angle));
	double zM = (liftJointHeight + upperArmLenght * sin(lift_joint_Angle) + forearmLenght * sin(lift_joint_Angle + elbow_joint_Angle) + endEffectorLenght * sin(lift_joint_Angle + elbow_joint_Angle + wrist1_joint_Angle));

	res.fPose.position.x = xM;
	res.fPose.position.y = yM;
	res.fPose.position.z = zM;

	return true;
}


bool Kinematics::do_ik (tr5_kinematics::DoInverseKinematic::Request  &req, tr5_kinematics::DoInverseKinematic::Response &res )
{
	//
	//compute inverse kinematics
	//

	double xM =  req.iPose.position.x, yM = req.iPose.position.y, zM = req.iPose.position.z;

	double pan_joint_Angle = atan2(yM, xM);
	pan_joint_Angle = degrees(pan_joint_Angle);

	if(pan_joint_Angle > 79) pan_joint_Angle = 80;
	else if(pan_joint_Angle < -79) pan_joint_Angle = -80;

	pan_joint_Angle = radians(pan_joint_Angle);

	double x = (pow(xM*cos(pan_joint_Angle) + yM*sin(pan_joint_Angle) - endEffectorLenght , 2) + pow(zM - liftJointHeight, 2) - upperArmLenght*upperArmLenght - forearmLenght*forearmLenght) / (2.0*upperArmLenght*forearmLenght);
	double sqrRoot = 1 - pow(x, 2);
	if (sqrRoot < 0) sqrRoot = 0;

	double y = sqrt(sqrRoot);

	double elbow_joint_Angle = atan2(y, x);
	if (elbow_joint_Angle > 0)	elbow_joint_Angle = atan2(-y, x);

	elbow_joint_Angle = degrees(elbow_joint_Angle);
	if(elbow_joint_Angle > -1)	elbow_joint_Angle = 0;
	else if(elbow_joint_Angle < -99) elbow_joint_Angle = -100;

	elbow_joint_Angle = radians(elbow_joint_Angle);

	x = (upperArmLenght + forearmLenght * cos(elbow_joint_Angle))*(-endEffectorLenght + cos(pan_joint_Angle)*xM + sin(pan_joint_Angle)*yM) + forearmLenght*sin(elbow_joint_Angle)*(-liftJointHeight + zM );
	y = (upperArmLenght + forearmLenght*cos(elbow_joint_Angle))*(-liftJointHeight + zM) - forearmLenght*sin(elbow_joint_Angle)*(cos(pan_joint_Angle)*xM + sin(pan_joint_Angle)*yM - endEffectorLenght);

	double lift_joint_Angle = atan2(y, x);

	lift_joint_Angle = degrees(lift_joint_Angle);
	if(lift_joint_Angle > 69) lift_joint_Angle = 70;
	else if(lift_joint_Angle < -29) lift_joint_Angle = -30;

	lift_joint_Angle = radians(lift_joint_Angle);

	double xG = cos(pan_joint_Angle) * (upperArmLenght * cos(lift_joint_Angle) + forearmLenght * cos(lift_joint_Angle + elbow_joint_Angle));
	double zG = (liftJointHeight + upperArmLenght * sin(lift_joint_Angle) + forearmLenght * sin(lift_joint_Angle + elbow_joint_Angle));

	double gXDist = xM - xG;
	double gZDist = zM - zG;

	double theta = 0;
	theta = atan2(gZDist, gXDist);
	double wrist1_joint_Angle = theta - lift_joint_Angle - elbow_joint_Angle;

	wrist1_joint_Angle = degrees(wrist1_joint_Angle);

	if(wrist1_joint_Angle > 99) wrist1_joint_Angle = 100;
	else if(wrist1_joint_Angle < -99) wrist1_joint_Angle = -100;

	wrist1_joint_Angle = radians(wrist1_joint_Angle);

	res.ijState.name.clear();
	res.ijState.name.push_back( "tr5shoulder_pan_joint"  );
	res.ijState.name.push_back( "tr5shoulder_lift_joint" );
	res.ijState.name.push_back( "tr5elbow_joint"         );
	res.ijState.name.push_back( "tr5wrist_1_joint"       );
	res.ijState.name.push_back( "tr5wrist_2_joint"       );
	res.ijState.name.push_back( "tr5gripper_1_joint"     );
	res.ijState.name.push_back( "tr5gripper_2_joint"     );
	res.ijState.position.push_back(pan_joint_Angle);
	res.ijState.position.push_back(lift_joint_Angle);
	res.ijState.position.push_back(elbow_joint_Angle);
	res.ijState.position.push_back(wrist1_joint_Angle);
	res.ijState.position.push_back(0);
	res.ijState.position.push_back(0);
	res.ijState.position.push_back(0);
	res.ijState.header.stamp = ros::Time::now();
	return true;
}

double Kinematics::degrees(double radians)
{
	return  radians / (M_PI / 180.0);
}

double Kinematics::radians(double degrees)
{
	return  degrees / (180.0 / M_PI);
}
