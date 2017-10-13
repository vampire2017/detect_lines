//
// Created by bpeer on 17-9-22.
//

#ifndef DETECT_LINES_DETECT_LINES_H
#define DETECT_LINES_DETECT_LINES_H

#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include "ransac_lines.h"
#include "std_msgs/Float64.h"
#include "bprobot/msg_Q_Wall_info.h"

class DetectLines
{
public:
	DetectLines();

private:
	void laserScanCb( const sensor_msgs::LaserScanConstPtr &laser_data);
	ros::NodeHandle nh_;

	ros::Subscriber laserScan_sub_;
	ros::Publisher marker_pub_;
	ros::Publisher detect_line_theta_pub_;

	double max_distance; //两个点之间的距离大于某一值，则作为不同物体的分界点
	int min_set_size; //判断是否为样本集的最小激光点的数量
	double disThre_loose; //松距离阈值判断
	double disThre_tight; //紧距离阈值判断
	int consensus_loose; //松一致性
	int consensus_tight; //紧一致性
	double min_length;  //设置墙壁(检测直线)的最小长度

};


#endif //DETECT_LINES_DETECT_LINES_H
