//
// Created by bpeer on 17-9-25.
//

#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <detect_lines/ransac_lines.h>
#include "std_msgs/Float64.h"

double laserScanLib(const sensor_msgs::LaserScanConstPtr &laser_data)
{
	double theta;
	bool flag;
//	RansacLines ransacLines( laser_data, 50, 2, 0.3, disThre_loose, consensus_loose, max_distance, min_set_size, disThre_tight, consensus_tight, min_length );
	RansacLines ransacLines( laser_data, 50, 2, 0.3, 0.05, 20, 0.5, 60, 0.02, 60, 1 );

	flag = ransacLines.getRansacLines();

	if( flag )
	{
		//输出一条最长的直线
		auto ind = ransacLines.lines.begin();
		for( auto i = ransacLines.lines.begin()+1; i != ransacLines.lines.end(); ++i )
		{
			if( i->get_length() > ind->get_length() )
				ind = i;
			std::cout << "检测到的直线长度:  " << i->get_length() << std::endl;
		}
		std::cout << "!!!最长直线的长度!!!:  " << ind->get_length() << std::endl;


		if( ind->get_b() == 0 )
			theta = M_PI/2;
		else
			theta = atan( -ind->get_a() / ind->get_b() );
		std::cout << "角度：  " << theta << std::endl;
		return theta;
	}
	else
		return NAN;

}