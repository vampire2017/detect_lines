//
// Created by bpeer on 17-9-22.
//

#include <detect_lines/ransac_lines.h>

/**
 * @brief 原始激光数据的处理与分割
 * @param laser_data
 * @param totalTrial	总的尝试次数:TotalTrial --> 50
 * @param totalSample	拟合初始模型所需直线样本数量:TotalSample --> 2
 * @param totalAngle	拟合初始模型选择样本的角度:TotalAngle --> 0.3
 * @param disThre		点到直线距离的阈值:DisThre --> disThre_loose 0.05
 * @param consensus		一致性数量:Consensus --> consensus_loose 20
 * @param maxDistance   分界点距离:MaxDistance --> max_distance 0.5
 * @param minSetSize    样本集的最小激光点的数量:MinSetSize --> min_set_size 60
 */
RansacLines::RansacLines(sensor_msgs::LaserScanConstPtr laser_data, int totalTrial, int totalSample, float totalAngle, float disThre, int consensus,
                         double maxDistance, int minSetSize, float disThreTight, int consensusTight, double minLength )
:laser_scan( laser_data ), TotalTrial( totalTrial ), TotalSample( totalSample ), TotalAngle( totalAngle ), DisThre( disThre ), Consensus( consensus ),
 MaxDistance( maxDistance ), MinSetSize( minSetSize ), DisThreTight( disThreTight ), ConsensusTight( consensusTight ), MinLength( minLength )
{
	set_laser_points();
	div_sample();

	NumIniSample = 5;
	ntrial = 0;
}

/**
 * @brief 激光数据的分割 --> 1)重新定义起点  2)按距离分割激光数据
 * @input laser_points 处理后的激光数据
 * @output sample_laser_points_cluster 样本数据集
 */
void RansacLines::div_sample()
{
	int div_start = laser_points.begin()->get_index();

	//find div start
	for( auto i = laser_points.begin(); i < laser_points.end()-1; ++i )
	{
		if( distance_between_point( *(i+1), *i )  > MaxDistance )
		{
			div_start = i - laser_points.begin() + 1;
			break;
		}
	}

	std::vector< LaserPoint > sample_laser_points;
	sample_laser_points.push_back( laser_points[div_start] );
	std::vector< LaserPoint >::iterator j;
	std::vector< LaserPoint >::iterator i = laser_points.begin() + div_start;
	for( int n = 0; n < laser_points.size(); ++n )
	{
		if( i == laser_points.end() )
			i = laser_points.begin();

		if( i == laser_points.end()-1 )
			j = laser_points.begin();
		else
			j = i + 1;

		if ( distance_between_point(*j, *i) < MaxDistance )
			sample_laser_points.push_back( *j );
		else
		{
			if( sample_laser_points.size() >= MinSetSize )
				sample_laser_points_cluster.push_back( sample_laser_points );

			sample_laser_points.clear();
			sample_laser_points.push_back( *j );  //every sample begin
		}

		++i;
	}

	//the last sample
	if( sample_laser_points.size() >= MinSetSize )
		sample_laser_points_cluster.push_back( sample_laser_points );

	sample_laser_points.clear();
}

/**
 * @brief 激光数据处理
 * @output laser_points
 */
void RansacLines::set_laser_points()
{
	for ( int i = 0; i < laser_scan->ranges.size(); i++ )
	{
		if ( laser_scan->ranges[i] >= laser_scan->range_min && laser_scan->ranges[i] <= laser_scan->range_max )
		{
			LaserPoint laserPoint( i, laser_scan);
			laser_points.push_back( laserPoint );
		}
	}
}

/**
 * @brief 获取拟合的直线
 * @return lines
 */
bool RansacLines::getRansacLines()
{
	bool flag = 0;

	for (auto i = sample_laser_points_cluster.begin(); i < sample_laser_points_cluster.end(); ++i)
	{
		laser_points.clear();
		laser_points.assign(i->begin(), i->end());
		int maxNumInlierPoints = 0;  //@todo test

		while (ntrial < TotalTrial && laser_points.size() > Consensus)
		{
			set_rnd_laser_points();  //@todo TotalSample = 2  所以拟合直线的a^2+b^2 != 0
			double d;
			Line line;
			line.fit_linesample.clear();
			line.fit_linesample.assign(rnd_laser_points.begin(), rnd_laser_points.end());

			line.fit_line();  //@todo 直线拟合有效性的判断  2个样本时不考虑均匀的情况  多个样本的时候可能需要考虑这种情况(圆拟合)
			for (auto j:laser_points)
			{
				d = line.dis_point2line(line.get_a(), j.get_x(), line.get_b(), j.get_y(), line.get_c());
				if (d < DisThre)  ///dis_thre_loose
					inline_laser_points.push_back(j);
				else
					outline_laser_points.push_back(j);
			}

			if (inline_laser_points.size() > Consensus)  ///consensus_loose
			{
				line.fit_linesample.clear();
				line.fit_linesample.assign( inline_laser_points.begin(), inline_laser_points.end() );
				bool loose_line_flag = line.fit_line();

				//增加一次迭代
				inline_laser_points.clear();
				outline_laser_points.clear();
				if (loose_line_flag)
				{
					for (auto k:laser_points)
					{
						d = line.dis_point2line(line.get_a(), k.get_x(), line.get_b(), k.get_y(), line.get_c());
						if (d < DisThreTight)  ///dis_thre_tight
							inline_laser_points.push_back(k);
						else
							outline_laser_points.push_back(k);
					}

					if (inline_laser_points.size() > ConsensusTight)
					{
						line.fit_linesample.clear();
						line.fit_linesample.assign(inline_laser_points.begin(), inline_laser_points.end());
						bool tight_line_flag = line.fit_line();
						if (tight_line_flag)
						{
							line.set_length();
							if (line.get_length() >  MinLength)
							{
								lines.push_back(line);
								find_line_from_outline_points();
								ntrial = 0;  //@todo 可以从-1出发
							}
						}
					}
				}
			}
			ntrial++;
			inline_laser_points.clear();
			outline_laser_points.clear();
		} ///end while
	} ///end for
	if( !lines.empty() )
		flag = 1;
	return flag;
}

/**
 * @brief 对于一个集合里的点,检测到一条直线之后,就把这条直线对应的点去掉,看集合里是否有其他直线
 * @input outline_laser_points
 * @output laser_points
 */
void RansacLines::find_line_from_outline_points()
{
	laser_points.clear();
	laser_points.assign( outline_laser_points.begin(), outline_laser_points.end() );
}

/**
 * @brief 随机生成样本激光点
 * @output rnd_laser_points
 */
void RansacLines::set_rnd_laser_points()
{
	rnd_laser_points.clear();
	int tmp = 0;
	//产生一个[NumIniSample, laser_points.size()-NumIniSample-1]之间的随机数
	int centerPoint = rand() % ( laser_points.size() - 2*NumIniSample ) + NumIniSample;
	bool newpoints;

	rnd_laser_points.push_back( laser_points[centerPoint] );
	for (int i = 1; i < TotalSample; ++i)
	{
		newpoints = false;
		while( !newpoints )
		{
			tmp = centerPoint + ( rand() % (2*NumIniSample) - NumIniSample );
			for (int j = 0; j < i; ++j)
			{
				if( rnd_laser_points[j].get_index() == laser_points[tmp].get_index() )
					break;

				if( j >= i-1 )
					newpoints = true;
			}
		}
		rnd_laser_points.push_back( laser_points[tmp] );
	}
}


