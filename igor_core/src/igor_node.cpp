#include <ros/ros.h>
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
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
#define NUMGOALS 4
#define NUMTARG 2

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

void odometryCB(const nav_msgs::Odometry::ConstPtr& msg, double* posx, double* posy) {
	printf("odometryCB\n");
	printf("%f %f\n", msg->pose.pose.position.x,msg->pose.pose.position.y);
	*posx = msg->pose.pose.position.x;
	*posy = msg->pose.pose.position.y;
	return;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "igor_node");
	ros::NodeHandle nc;
	
	ros::Subscriber sub, sub2, sub3;
	int lentarg = min(NUMTARG, NUMGOALS);
	
	bool must_stop = false;
	int nocent = 0;
	//geometry_msgs::PoseStamped odom;
	double posx,posy;
	
	MoveBaseClient ac("move_base", true);
	
	// Wait 60 seconds for the action server to become available
	ROS_INFO("[INFO] Waiting for the move_base action server");
	ac.waitForServer(ros::Duration(60));
	
	ROS_INFO("[INFO] Connected to move base server");
	
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

	//----------------------------------------------------------

	goals[2].target_pose.header.frame_id = "map";
	goals[2].target_pose.header.stamp = ros::Time::now();
	goals[2].target_pose.pose.position.x = 13.2;
	goals[2].target_pose.pose.position.y = 0.7;
	theta = 0;
	radians = theta * (M_PI/180);
	quaternion = tf::createQuaternionFromYaw(radians);
	tf::quaternionTFToMsg(quaternion, qMsg);
	goals[2].target_pose.pose.orientation = qMsg;
	
	//----------------------------------------------------------

	goals[3].target_pose.header.frame_id = "map";
	goals[3].target_pose.header.stamp = ros::Time::now();
	goals[3].target_pose.pose.position.x = 13.1;
	goals[3].target_pose.pose.position.y = 2.3;
	theta = 90;
	radians = theta * (M_PI/180);
	quaternion = tf::createQuaternionFromYaw(radians);
	tf::quaternionTFToMsg(quaternion, qMsg);
	goals[3].target_pose.pose.orientation = qMsg;
	
//###################################################################################	
	
	ros::Publisher pub_vel = nc.advertise<geometry_msgs::Twist>("cmd_vel", 100);
	while(1) {
		sub = nc.subscribe<std_msgs::String>("/detector/feedback", 100, boost::bind(detectionCB, _1, &must_stop));
		while(!must_stop) ros::spinOnce();
		must_stop = false;
		sub.shutdown();
		ROS_INFO("[INFO] Cancelling goals");
		ac.cancelAllGoals();
		sub2 = nc.subscribe<std_msgs::String>("/detector/centre", 100, boost::bind(followCB, _1, pub_vel, &nocent));
		ROS_INFO("[INFO] Subscribed to sub2");
		while(nocent < MOVELIM) ros::spinOnce();
		nocent = 0;
		sub2.shutdown();		
		ROS_INFO("[OK] Target reached, commencing task");
		
		
		ROS_INFO("[OK] B4 sub3");		
		//sub3 = nc.subscribe<nav_msgs::Odometry>("/odom", 10, boost::bind(odometryCB, _1, &posx, &posy));
		//ros::spinOnce();
		ROS_INFO("[OK] After sub3");
		//printf("%f %f\n", odom.pose.position.x, odom.pose.position.y);
		printf("%f %f\n", posx, posy);
		
		
		double dist;
		int aux;
		geometry_msgs::PoseStamped test;
		move_base_msgs::MoveBaseGoal copygoals[NUMGOALS];
		for(int j=0; j<NUMGOALS; ++j) {
			copygoals[j].target_pose.header.frame_id = "map";
			copygoals[j].target_pose.header.stamp = ros::Time::now();
			copygoals[j].target_pose.pose.position.x = goals[j].target_pose.pose.position.x;
			copygoals[j].target_pose.pose.position.y = goals[j].target_pose.pose.position.y;
			copygoals[j].target_pose.pose.orientation = goals[j].target_pose.pose.orientation;
		}
		
		for(int j=1; j<NUMGOALS; ++j) {
			test.header.frame_id = "map";
			test.header.stamp = ros::Time::now();
			test.pose.position.x = copygoals[j].target_pose.pose.position.x;
			test.pose.position.y = copygoals[j].target_pose.pose.position.y;
			test.pose.orientation = copygoals[j].target_pose.pose.orientation;
			//dist = sqrt(pow(copygoals[j].target_pose.pose.position.x - odom.pose.position.x, 2) + pow(copygoals[j].target_pose.pose.position.y - odom.pose.position.y, 2));
			dist = sqrt(pow(copygoals[j].target_pose.pose.position.x - posx, 2) + pow(copygoals[j].target_pose.pose.position.y - posy, 2));
			aux = j - 1;
			//while(aux >= 0 && sqrt(pow(copygoals[aux].target_pose.pose.position.x - odom.pose.position.x, 2) + pow(copygoals[aux].target_pose.pose.position.y - odom.pose.position.y, 2)) > dist) {
			while(aux >= 0 && sqrt(pow(copygoals[aux].target_pose.pose.position.x - posx, 2) + pow(copygoals[aux].target_pose.pose.position.y - posy, 2)) > dist) {
				copygoals[aux + 1].target_pose.header.frame_id = "map";
				copygoals[aux + 1].target_pose.header.stamp = ros::Time::now();
				copygoals[aux + 1].target_pose.pose.position.x = goals[aux].target_pose.pose.position.x;
				copygoals[aux + 1].target_pose.pose.position.y = goals[aux].target_pose.pose.position.y;
				copygoals[aux + 1].target_pose.pose.orientation = goals[aux].target_pose.pose.orientation;
				aux--;
			}
			copygoals[aux + 1].target_pose.header.frame_id = "map";
			copygoals[aux + 1].target_pose.header.stamp = ros::Time::now();
			copygoals[aux + 1].target_pose.pose.position.x = test.pose.position.x;
			copygoals[aux + 1].target_pose.pose.position.y = test.pose.position.y;
			copygoals[aux + 1].target_pose.pose.orientation = test.pose.orientation;
		}
		
		for(int i=0; i<NUMGOALS; ++i) {
			//printf("[%f, %f, %f] ", copygoals[i].target_pose.pose.position.x, copygoals[i].target_pose.pose.position.y, sqrt(pow(copygoals[i].target_pose.pose.position.x - odom.pose.position.x, 2) + pow(copygoals[i].target_pose.pose.position.y - odom.pose.position.y, 2)));
			printf("[%f, %f, %f] ", copygoals[i].target_pose.pose.position.x, copygoals[i].target_pose.pose.position.y, sqrt(pow(copygoals[i].target_pose.pose.position.x - posx, 2) + pow(copygoals[i].target_pose.pose.position.y - posy, 2)));
		}
		printf("\n");
		for(int i=0; i<NUMGOALS; ++i) {
			printf("[%f, %f] ", goals[i].target_pose.pose.position.x, goals[i].target_pose.pose.position.y);
		}
		printf("\n");
		return 0;
		
		
		int i = 0;
		while(1){
			ROS_INFO("[INFO] Goal #%d: Sending IGOR to position [%f, %f: %f]", i, copygoals[i].target_pose.pose.position.x, copygoals[i].target_pose.pose.position.y, copygoals[i].target_pose.pose.position.z);
			ac.sendGoal(copygoals[i]);

			// Wait for the action to return
			ac.waitForResult();

			if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				ROS_INFO("[OK] Position [%f, %f: %f] reached", copygoals[i].target_pose.pose.position.x, copygoals[i].target_pose.pose.position.y, copygoals[i].target_pose.pose.position.z);
				ros::Duration(5).sleep();
			}
			else {
				ROS_INFO("[FAIL] The base failed for some reason");
				break;
			}
			i = (i+1)%NUMTARG;
		}
	}		
}
