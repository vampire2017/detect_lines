//
// Created by bpeer on 17-9-22.
//
#ifndef DETECT_LINES_LINE_H
#define DETECT_LINES_LINE_H

#include <iostream>
#include "laser_point.h"

float distance_between_point( LaserPoint& p1, LaserPoint& p2 );

class  Line
{
public:
	Line(){};
	bool fit_line();
	double dis_point2line( const float a, const float x, const float b, const float y, const float c );
	void set_length();

	float get_a();
	float get_b();
	float get_c();
	float get_length();
	float get_line_score();

	std::vector< LaserPoint > fit_linesample;

private:
	float a;
	float b;
	float c;            //ax+by+c = 0
	float length;
	float line_score;

};

#endif //DETECT_LINES_LINE_H
