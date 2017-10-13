//
// Created by bpeer on 17-9-22.
//

#ifndef DETECT_LINES_LASER_POINT_H
#define DETECT_LINES_LASER_POINT_H

#include "sensor_msgs/LaserScan.h"

class LaserPoint
{
public:
	LaserPoint( int ind, const sensor_msgs::LaserScanConstPtr& scan)
	{
		index = ind;
		range = scan->ranges[ind];

		x = (float)(cos( scan->angle_min + ind * scan->angle_increment ) * range);
		y = (float)(sin( scan->angle_min + ind * scan->angle_increment ) * range);
	};

	int   get_index()       { return index; };
	float get_range()       { return range; };
	float get_intensity()   { return intensity; };
	float get_x()           { return x; };
	float get_y()           { return y; };

private:
	int   index;
	float range;
	float intensity;
	float x;
	float y;
};


#endif //DETECT_LINES_LASER_POINT_H
