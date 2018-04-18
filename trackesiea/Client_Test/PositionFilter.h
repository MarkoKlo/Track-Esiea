#pragma once
#include<opencv2\opencv.hpp>

using namespace cv;

class LowPassFilter
{
public:
	LowPassFilter();
	~LowPassFilter();
	void LowPassFilter::filter(Point3f& output);

private:
	bool m_firstTime;
	float alpha;

	Point3f m_position;
	Point3f m_lastPosition;

	void LowPassFilter::clampAlpha(float& alpha);
};

class OneEuroFilter
{
public:
	OneEuroFilter();
	~OneEuroFilter();
	void OneEuroFilter::filter();

private:
	bool m_firstTime;
	float m_rate;
	float m_minCutoff;
	float beta;
	float m_dCutoff;

	Point3f m_position;
	Point3f m_lastPosition;
	
	void OneEuroFilter::lowPass(Point3f& output);
};

