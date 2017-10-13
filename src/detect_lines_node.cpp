/**
 * @brief detect lines
 * @time 2017年09月22日14:12:33
 * @author vampire
 */

#include <detect_lines/detect_lines.h>

int main( int argc, char** argv)
{
	ros::init(argc, argv, "detect_lines_node");

	DetectLines detectLines;

	ros::spin();
	return 0;
}