//
// Created by bpeer on 17-9-22.
//
#include <detect_lines/line.h>

float distance_between_point( LaserPoint& p1, LaserPoint& p2)
{
	float dis2_ = ( p1.get_x()-p2.get_x() ) * ( p1.get_x()-p2.get_x() ) + ( p1.get_y()-p2.get_y() ) * ( p1.get_y()-p2.get_y() );
	return sqrt(dis2_);
}

void Line::set_length()
{
	length = distance_between_point( *fit_linesample.begin(), *(fit_linesample.end()-1) );
}

/**
 * @brief 拟合直线
 * @return
 */
bool Line::fit_line()
{
	int size = fit_linesample.size();
	if( size < 2 )
	{
		a = 0;
		b = 0;
		c = 0;
		return false;
	}

	float x_mean = .0;
	float y_mean = .0;

	for( auto i:fit_linesample )
	{
		x_mean += i.get_x();
		y_mean += i.get_y();
	}
	x_mean /= size;
	y_mean /= size;  //至此，计算出了 x y 的均值

	float Dxx = 0, Dxy = 0, Dyy = 0;

	for( auto i:fit_linesample )
	{
		Dxx += ( i.get_x() - x_mean ) * ( i.get_x() - x_mean );
		Dxy += ( i.get_x() - x_mean ) * ( i.get_y() - y_mean );
		Dyy += ( i.get_y() - y_mean ) * ( i.get_y() - y_mean );
	}

	/// lambda = ( Dxx+Dyy - sqrt((Dxx-Dyy)^2 + 4*Dxy^2) )/2
	float lambda = ( (Dxx + Dyy) - sqrt( (Dxx - Dyy) * (Dxx - Dyy) + 4 * Dxy * Dxy ) ) / 2.0;

	/// den = sqrt( Dxy^2 + (lambda-Dxx)^2 )
	float den = sqrt( Dxy * Dxy + (lambda - Dxx) * (lambda - Dxx) );

	/**
	 * @brief http://blog.csdn.net/liyuanbhu/article/details/51137038
	 * den = 0 的情况
	 * Dxy=0 说明xi 和yi是线性无关的。再往下还可以细分为 2 种情况。
	 * Dyy!=Dxx 这时数据点落在一条垂直直线上，直线方程为 x=x¯。
	 * Dyy=Dxx 数据均匀分布，找不到特殊的方向，无法拟合直线方程。
	 */
	if( fabs(den) < 1e-5 )
	{
		if( fabs(Dxx / Dyy - 1) < 1e-5 )
		{
			return false;
		}
		else
		{
			a = 1;
			b = 0;
			c = - x_mean;
		}
	}
	else
	{
		a = Dxy / den;
		b = (lambda - Dxx) / den;
		c = -a * x_mean - b * y_mean;
	}
	return true;
}

float  Line::get_a()
{
	return a;
}

float Line::get_b()
{
	return b;
}

float Line::get_c()
{
	return c;
}

float Line::get_length()
{
	return length;
}

float Line::get_line_score()
{
	return line_score;
}

double Line::dis_point2line(const float a, const float x, const float b, const float y, const float c)
{
	return fabs( a * x + b * y + c ) / sqrt( a * a + b * b );
}
