/***************************** Made by Duarte Gonçalves and André Lourenço  *********************************/
#include "line_detector/LineDetector.hpp"


LineDetector::LineDetector(){
	m_sub_intensitySensor = nh2.subscribe("/nxt/intensity_sensor", 1, &LineDetector::IntensitySensorClbk, this );
	m_pub_lineInSight = nh2.advertise<std_msgs::Bool>("/nxt/line_in_sight", 1);
	m_pub_Go = nh2.advertise<std_msgs::Bool>("/nxt/goNxt", 1);
	m_pub_Intensity = nh2.advertise<std_msgs::Float64>("/nxt/LineIntesity", 1);
	isCalibrated = false;
}

void LineDetector::run(){
	ros::spin();
}

void LineDetector::IntensitySensorClbk( const nxt_msgs::Light msg){
	msgIntensity = msg.intensity;
	std_msgs::Bool go;
	if(isCalibrated){ //Finished Calibration
		go.data = true;
		m_pub_Go.publish(go);
		if(msgIntensity > intensityCalibrated){
			blackLineDetected.data = false;
		}
		else{
			blackLineDetected.data = true;
		}
		intens.data = msgIntensity;
		m_pub_Intensity.publish(intens);
		m_pub_lineInSight.publish(blackLineDetected);
	}
	else{ //Calibration
		go.data = false;
		m_pub_Go.publish(go);
		for(int i=0;i<10000;i++) //Don't do nothing! Its calibrating
		intensityCalibrated = msgIntensity;
		isCalibrated = true;
	}
}
