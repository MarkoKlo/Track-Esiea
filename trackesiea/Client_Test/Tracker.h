#pragma once
#include<iostream>
#include<cstdio>
#include<math.h>
#include<opencv2\opencv.hpp>

#define PI 3.1415926536
#define BALL_RADIUS 2.0
#define PSEYE_FOCAL 550.0

class Tracker
{
public:
	Tracker::Tracker(float ballRadius, float cameraFocalLength, cv::Point3f* position, cv::Point3f* speed);
	cv::Mat videoFrame;
	cv::Point3f* m_position;
	cv::Point3f* m_speed;
	cv::Mat filteredFrame;
	void init_tracker(int cameraIndex,bool stereo);
	void track();
	void set_filter_color(cv::Scalar color);
	void set_filter_range(cv::Vec3i hsvrange);

private :
	
	VideoCapture m_videoCap;
	float m_focalLength;
	float m_ballRadius;
	cv::Vec3i m_hsvRange;
	cv::Scalar m_filterColor;
	Point3f m_lastPosition;
	void color_filtering();
	void circle_fitting(cv::Point3f& circleCoord);
	void mono_position_estimation(cv::Point3f circleCoord, cv::Point3f& outPosition);
	std::vector<cv::Point> get_largest_contour(std::vector<std::vector<cv::Point> > contours);
};

