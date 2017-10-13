//
// Created by bpeer on 17-9-25.
//

#include <detect_lines/libDetectLineTheta.h>
#include "iostream"
#include "ros/ros.h"

void laserCb( const sensor_msgs::LaserScanConstPtr &laserScan )
{
	double test_theta = laserScanLib( laserScan );

	ROS_WARN("test.... %lf", test_theta);
}

int main( int argc, char** argv )
{
	ros::init( argc, argv, "test_lib_node" );
	ros::NodeHandle nh_;
	ros::Subscriber laserSub = nh_.subscribe("/scan", 1 , laserCb);

	ros::spin();
	return 0;
}
