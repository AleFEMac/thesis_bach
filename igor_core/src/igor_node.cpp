#include <ros/ros.h>
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <stdlib.h>
#include <QTransform>
#include <cmath>

#define DEVHEIGHT 480
#define DEVWIDTH 640
#define THRESHOLD 0.01
#define MAXVEL 0.5
#define MOVELIM 20
#define NUMGOALS 2

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void detectionCB(const std_msgs::String::ConstPtr& msg, bool* must_stop) {
	//ROS_INFO("detectionCB");
	if(!(msg->data == "")) {
		*must_stop = true;
		ROS_INFO("Object detected");
	}
}

void followCB(const std_msgs::String::ConstPtr& msg, ros::Publisher pub_vel,int* nocent) {
	//ROS_INFO("followCB");
	if(msg->data == "") (*nocent)++;
	else {
		double dcx;
		geometry_msgs::Twist tmsg;
		
		*nocent = 0;		
		dcx = stod(msg->data);		
		if(abs(dcx) < THRESHOLD) return;
		
		//Linear velocity
		tmsg.linear.x = (MAXVEL - abs(dcx))/4;
		tmsg.linear.y = 0;
		tmsg.linear.z = 0;
		
		//Angular velocity
		tmsg.angular.x = 0;
		tmsg.angular.y = 0;
		tmsg.angular.z = dcx;		
		
		pub_vel.publish(tmsg);
		ros::spinOnce();
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "igor_node");
	ros::NodeHandle nc;
	bool must_stop = false;
	int nocent = 0;
	geometry_msgs::PoseStamped odom;
	
	MoveBaseClient ac("move_base", true);
	
	// Wait 60 seconds for the action server to become available
	ROS_INFO("Waiting for the move_base action server");
	ac.waitForServer(ros::Duration(60));
	
	ROS_INFO("Connected to move base server");
	
//###################################################################################	
	
	move_base_msgs::MoveBaseGoal goals[NUMGOALS];
	
	tf::Quaternion quaternion;
	geometry_msgs::Quaternion qMsg;
	double radians, theta;
	
	goals[0].target_pose.header.frame_id = "map";
	goals[0].target_pose.header.stamp = ros::Time::now();
	goals[0].target_pose.pose.position.x = -0.865;
	goals[0].target_pose.pose.position.y = -1.03;
	theta = -120;
	radians = theta * (M_PI/180);
	quaternion = tf::createQuaternionFromYaw(radians);
	tf::quaternionTFToMsg(quaternion, qMsg);
	goals[0].target_pose.pose.orientation = qMsg;

	//----------------------------------------------------------

	goals[1].target_pose.header.frame_id = "map";
	goals[1].target_pose.header.stamp = ros::Time::now();
	goals[1].target_pose.pose.position.x = 7.44;
	goals[1].target_pose.pose.position.y = 0.0427;
	theta = -110;
	radians = theta * (M_PI/180);
	quaternion = tf::createQuaternionFromYaw(radians);
	tf::quaternionTFToMsg(quaternion, qMsg);
	goals[1].target_pose.pose.orientation = qMsg;
	
//###################################################################################	
	
	ros::Publisher pub_vel = nc.advertise<geometry_msgs::Twist>("cmd_vel", 100);
	ros::Subscriber sub;
	ros::Subscriber sub2;
	while(1) {
		sub = nc.subscribe<std_msgs::String>("/detector/feedback", 100, boost::bind(detectionCB, _1, &must_stop));
		while(!must_stop) ros::spinOnce();
		must_stop = false;
		sub.shutdown();
		ROS_INFO("Cancelling goals");
		ac.cancelAllGoals();
		sub2 = nc.subscribe<std_msgs::String>("/detector/centre", 100, boost::bind(followCB, _1, pub_vel, &nocent));
		ROS_INFO("Subscribed to sub2");
		while(nocent < MOVELIM) ros::spinOnce();
		nocent = 0;
		sub2.shutdown();
		
		
		
		ROS_INFO("Target reached, commencing task");
		//else {
			int i = 0;
			while(1){
				ROS_INFO("[INFO] Goal #%d: Sending IGOR to position [%f, %f: %f]", i, goals[i].target_pose.pose.position.x, goals[i].target_pose.pose.position.y, goals[i].target_pose.pose.position.z);
				ac.sendGoal(goals[i]);

				// Wait for the action to return
				ac.waitForResult();

				if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
					ROS_INFO("[OK] Position [%f, %f: %f] reached", goals[i].target_pose.pose.position.x, goals[i].target_pose.pose.position.y, goals[i].target_pose.pose.position.z);
					ros::Duration(5).sleep();
				}
				else {
					ROS_INFO("[FAIL] The base failed for some reason");
					break;
				}
				i = (i+1)%NUMGOALS;
			}
		//}
	}		
}
