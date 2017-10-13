//
// Created by bpeer on 17-9-22.
//

#ifndef DETECT_LINES_RANSAC_LINES_H
#define DETECT_LINES_RANSAC_LINES_H

#include "detect_lines.h"
#include "laser_point.h"
#include "line.h"

class RansacLines
{
public:
	RansacLines(){};
	RansacLines( sensor_msgs::LaserScanConstPtr laser_data, int totalTrial, int totalSample, float totalAngle, float disThre, int consensus,
				 double maxDistance, int minSetSize, float disThreTight, int consensusTight, double minLength );

	void set_laser_points();
	void div_sample();
	bool getRansacLines();  //检测直线;
	void set_rnd_laser_points();
	void find_line_from_outline_points();  //检测外点中是否还有其他直线

	sensor_msgs::LaserScanConstPtr laser_scan;
	std::vector< Line > lines;

	~RansacLines(){};
private:
	std::vector< std::vector< LaserPoint > > sample_laser_points_cluster;
	std::vector< LaserPoint > laser_points;
	std::vector< LaserPoint > rnd_laser_points;
	std::vector< LaserPoint > inline_laser_points;
	std::vector< LaserPoint > outline_laser_points;

	double MaxDistance;     //两个点之间的距离大于某一值，则作为不同物体的分界点
	int MinSetSize;         //样本集的最小激光点的数量
	float DisThreTight;     //样本集的最小激光点的数量
	int ConsensusTight;     //紧一致性数量
	double MinLength;        //设置墙壁(检测直线)的最小长度

	int ntrial;

	int TotalTrial;     //总的尝试次数
	int TotalSample;    //拟合初始模型所需直线样本数量
	float TotalAngle;   //拟合初始模型选择样本的角度
	float DisThre;      //点到直线距离的阈值
	int Consensus;      //松一致性数量
	int NumIniSample;   //若数据全部有效，采样范围内含的点数

};

#endif //DETECT_LINES_RANSAC_LINES_H
