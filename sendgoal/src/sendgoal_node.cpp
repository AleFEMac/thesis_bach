#include <ros/ros.h>
#include <string>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <stdlib.h>

#define NUMGOALS 4

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv) {
	ros::init(argc, argv, "send_goals_node");

	// create the action client
	// true causes the client to spin its own thread
	MoveBaseClient ac("move_base", true);

	// Wait 60 seconds for the action server to become available
	ROS_INFO("Waiting for the move_base action server");
	ac.waitForServer(ros::Duration(60));

	ROS_INFO("Connected to move base server");

	// Send a goal to move_base
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

	return 0;
}
