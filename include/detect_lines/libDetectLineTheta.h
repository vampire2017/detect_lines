//
// Created by bpeer on 17-9-25.
//

#ifndef DETECT_LINES_LIBDETECTLINETHETA_H
#define DETECT_LINES_LIBDETECTLINETHETA_H

#include "sensor_msgs/LaserScan.h"

//return line theta : rad   -->   (-1.57 ~ 1.57) & NAN
double laserScanLib(const sensor_msgs::LaserScanConstPtr &laser_data);

#endif //DETECT_LINES_LIBDETECTLINETHETA_H
