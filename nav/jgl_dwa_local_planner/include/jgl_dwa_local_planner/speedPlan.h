#include<iostream>
#include <vector>
//#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <math.h>

class SpeedPlan{
public:
	SpeedPlan(double vmax,double am);
	void  speedComputeLine(double d,double length,std::vector<double>&fov_speed,const float map_resolution);

	double vmax;
	double amax;

	
};