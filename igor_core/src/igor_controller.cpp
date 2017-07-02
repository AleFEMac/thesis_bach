#include <ros/ros.h>
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <stdlib.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void detectionCB(const std_msgs::String::ConstPtr& msg, bool* must_stop) {
	//ROS_INFO("detectionCB");
	if(!(msg->data == "")) {
		*must_stop = true;
		ROS_INFO("Object detected");
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "igor_controller");
	ros::NodeHandle nc;
	bool must_stop = false;
	
	MoveBaseClient ac("move_base", true);
	
	// Wait 60 seconds for the action server to become available
	ROS_INFO("Waiting for the move_base action server");
	ac.waitForServer(ros::Duration(60));
	
	ROS_INFO("Connected to move base server");
	
	ros::Subscriber sub = nc.subscribe<std_msgs::String>("/detector/feedback", 100, boost::bind(detectionCB, _1, &must_stop));
	while(1) {
		while(!must_stop) ros::spinOnce();
		ac.cancelAllGoals();
		must_stop = false;
	}
}
