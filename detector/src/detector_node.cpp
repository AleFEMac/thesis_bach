#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include <string>
#include <sstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <QTransform>
#include <cmath>

#define DEVHEIGHT 480
#define DEVWIDTH 640
#define NOOBJSTR ""

using namespace std;

void detectionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg, ros::Publisher pub, ros::Publisher pub3) {
	std_msgs::String feed, cent;
	std::stringstream ss, ss1;
	ros::Time t = ros::Time::now();
	geometry_msgs::Twist tmsg;
	double cx, cy, dcx ;
	
	if(msg->data.size()) {
		
		ss << "Object " << msg->data[0] << " located at " << t.sec << "." << t.nsec;
		feed.data = ss.str();
		pub.publish(feed);
		
		// get data
		int id = (int)msg->data[0];
		float objectWidth = msg->data[1];
		float objectHeight = msg->data[2];

		// Find corners Qt
		QTransform qtHomography(msg->data[3], msg->data[4], msg->data[5], msg->data[6], msg->data[7], msg->data[8], msg->data[9], msg->data[10], msg->data[11]);

		QPointF qtTopLeft = qtHomography.map(QPointF(0,0));
		QPointF qtTopRight = qtHomography.map(QPointF(objectWidth,0));
		QPointF qtBottomLeft = qtHomography.map(QPointF(0,objectHeight));
		QPointF qtBottomRight = qtHomography.map(QPointF(objectWidth,objectHeight));

		printf("\nObject %d detected, Qt corners at (%f,%f) (%f,%f) (%f,%f) (%f,%f)\n",
		id,
		qtTopLeft.x(), qtTopLeft.y(),
		qtTopRight.x(), qtTopRight.y(),
		qtBottomLeft.x(), qtBottomLeft.y(),
		qtBottomRight.x(), qtBottomRight.y());
		
		//Centre of the box
		cx = (min(qtTopLeft.x(), qtBottomLeft.x()) + max(qtTopRight.x(), qtBottomRight.x()))/2;
		cy = (min(qtBottomLeft.y(), qtBottomRight.y()) + max(qtTopLeft.y(), qtTopRight.y()))/2;
		printf("centre: [%f, %f]\n", cx, cy);
		
		dcx = (DEVWIDTH/2 - cx)/DEVWIDTH;
		printf("dcx = %f\n", dcx);
		
		ss1 << dcx;
		cent.data = ss1.str();
		pub3.publish(cent);

	} else {
		ss << NOOBJSTR;
		feed.data = ss.str();
		pub.publish(feed);
		
		ss1 << "";
		cent.data = ss1.str();
		pub3.publish(cent);
	}
	
	ros::spinOnce();
	
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "detector_node");

	ros::NodeHandle n;
	
	ros::Publisher pub = n.advertise<std_msgs::String>("detector/feedback", 100);
	ros::Publisher pub3 = n.advertise<std_msgs::String>("detector/centre", 100);
	ros::Subscriber sub = n.subscribe<std_msgs::Float32MultiArray>("objects", 100, boost::bind(detectionCallback, _1, pub, pub3));
	ros::spin();
	
	return 0;
}

