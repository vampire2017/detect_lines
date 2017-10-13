//
// Created by bpeer on 17-9-22.
//

#include <detect_lines/detect_lines.h>

DetectLines::DetectLines()
{

	ros::NodeHandle private_nh_("~");
	private_nh_.param< double >("max_distance", max_distance, 0.5);
	private_nh_.param< int >("min_set_size", min_set_size, 60);
	private_nh_.param< double >("disThre_loose", disThre_loose, 0.05);
	private_nh_.param< double >("disThre_tight", disThre_tight, 0.02);
	private_nh_.param< int >("consensus_loose", consensus_loose, 20);
	private_nh_.param< int >("consensus_tight", consensus_tight, 60);
	private_nh_.param< double >("min_length", min_length, 1);

	laserScan_sub_ = nh_.subscribe("/scan", 10, &DetectLines::laserScanCb, this);
	marker_pub_ = nh_.advertise< visualization_msgs::Marker >("visualization_marker", 10);
	detect_line_theta_pub_ = nh_.advertise< bprobot::msg_Q_Wall_info >("msg_Q_Wall_info", 2);
}

void DetectLines::laserScanCb(const sensor_msgs::LaserScanConstPtr &laser_data)
{
	double theta;
	bool flag;
	RansacLines ransacLines( laser_data, 50, 2, 0.3, disThre_loose, consensus_loose, max_distance, min_set_size, disThre_tight, consensus_tight, min_length );

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

		visualization_msgs::Marker marker;
		marker.header.frame_id = "/laser_frame";
		marker.header.stamp = ros::Time::now();
//		marker.ns = "lines";
//		marker.id = 0;
//		marker.type = visualization_msgs::Marker::LINE_STRIP;
//		marker.action = visualization_msgs::Marker::ADD;
//
//		marker.pose.orientation.w = 1.0;
//		marker.scale.x = 0.01;
//		marker.scale.y = 0.01;
//		marker.color.r = 0.0f;
//		marker.color.b = 1.0f;
//		marker.color.a = 1.0f;
//
		geometry_msgs::Point p;
//
//		for( auto j:ind->fit_linesample )
//		{
//			p.x = j.get_x();
//			p.y = j.get_y();
//			p.z = 0;
//			marker.points.push_back( p );
//		}
//		marker_pub_.publish( marker );
//		marker.points.clear();

		marker.ns = "theLine";
		marker.id = 0;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.orientation.w = 1.0;
		marker.type = visualization_msgs::Marker::LINE_LIST;
		marker.scale.x = 0.01;
		marker.scale.y = 0.01;
		marker.color.r = 1.0f;
		marker.color.b = 0.0f;
		marker.color.g = 1.0f;
		marker.color.a = 1.0f;

		if( ind->get_b() == 0 )
		{
			p.x = -ind->get_c() / ind->get_a();
			p.y = ind->fit_linesample.begin()->get_y();
			p.z = 0;
			marker.points.push_back( p );
			p.x = -ind->get_c() / ind->get_a();
			p.y = ( ind->fit_linesample.end()-1 )->get_y();
			p.z = 0;
			marker.points.push_back( p );
		}
		else
		{
			p.x = ind->fit_linesample.begin()->get_x();
			p.y = ( -ind->get_c() - ind->get_a() * p.x ) / ind->get_b();
			p.z = 0;
			marker.points.push_back( p );
			p.x = ( ind->fit_linesample.end()-1 )->get_x();
			p.y = ( -ind->get_c() - ind->get_a() * p.x ) / ind->get_b();
			p.z = 0;
			marker.points.push_back( p );
		}
		marker_pub_.publish( marker );
		marker.points.clear();

		if( ind->get_b() == 0 )
			theta = M_PI/2;
		else
			theta = atan( -ind->get_a() / ind->get_b() );
		std::cout << "角度：  " << theta << std::endl;

		bprobot::msg_Q_Wall_info line_theta_msg;
		line_theta_msg.stamp = ros::Time::now();
		line_theta_msg.Detect_Type = 1;
		line_theta_msg.Angle_Deg = theta;
		detect_line_theta_pub_.publish( line_theta_msg );

	}
	else
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/laser_frame";
		marker.header.stamp = ros::Time::now();

		geometry_msgs::Point p;

		marker.ns = "theLine";
		marker.id = 0;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.orientation.w = 1.0;
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.scale.x = 0.01;
		marker.scale.y = 0.01;
		marker.color.r = 1.0f;
		marker.color.b = 0.0f;
		marker.color.g = 1.0f;
		marker.color.a = 1.0f;
//		marker.points.push_back( p );

		marker_pub_.publish( marker );
	}
}
