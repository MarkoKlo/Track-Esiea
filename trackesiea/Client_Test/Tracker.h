#pragma once
#include<iostream>
#include<cstdio>
#include<math.h>
#include<opencv2\opencv.hpp>
#include<opencv2\world.hpp>
#include<libusb.h>

#define PI 3.1415926536
#define BALL_RADIUS 2.0
#define PSEYE_FOCAL 550.0

#define LOWPASS_ALPHA 0.5
#define Z_LOWPASS_SMOOTHING 0.5

/*
Cette classe a pour rôle d'effectuer le tracking 3D de la sphère.
*/

using namespace cv;

//Membres de la classe Tracker
class Tracker
{
public:
	Tracker::Tracker();
	Tracker::Tracker(float ballRadius, float cameraFocalLength);
	Tracker::~Tracker();
	void init_tracker(int cameraIndex,bool stereo);
	void track();
	void set_filter_color(Vec3i color);
	void set_filter_range(Vec3i hsvrange);
	Vec3i get_hsv_color(Point2i coordinates);
	Vec3i get_filter_color();
	Vec3i get_filter_range();
	Point3f get_2D_position();
	Point3f get_position();
	Point3f get_speed();
	bool is_tracking_valid();

	Mat& get_video_frame();
	Mat& get_binary_frame();

	enum filterType{simple_lowpass,multi_channel_lowpass,noFiltering};
	filterType m_filteringType;

	void set_delta_time(int64* m_currentTick,int64* las);

private :

	// Variables accessibles via les fonctions d'accès
	VideoCapture m_videoCap;
	Mat m_videoFrame;
	Mat m_hsvFrame;
	Mat m_filteredFrame;
	Point3f m_2Dposition;
	Point3f m_position;
	Point3f m_speed;
	bool m_isTrackingValid;

	// Réglages
	float m_focalLength;
	float m_ballRadius;
	Vec3i m_hsvRange;
	Vec3i m_filterColor;

	// Variables privées
	Point3f* m_lastPosition;
	double m_deltaTime;
	int64* m_currentTick;
	int64* m_lastTick;

	// Fonctions privées
	void color_filtering(Mat& videoFrame, Vec3i hsvRange, Scalar filterColor, Mat& filteredFrame);
	void circle_fitting(Point3f& circleCoord, Mat& filteredFrame);
	void mono_position_estimation(float focal, Point3f circleCoord, Point3f& outPosition);
	std::vector<Point> get_largest_contour(std::vector<std::vector<Point> > contours);
	
};

