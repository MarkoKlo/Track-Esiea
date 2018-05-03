#include "PositionFilter.h"

LowPassFilter::LowPassFilter()
{
}

LowPassFilter::~LowPassFilter()
{
}

void LowPassFilter::filter(Point3f & output)
{
	if (this->m_firstTime)
	{
		m_firstTime = false;
		m_lastPosition = m_position;
	}
}

void LowPassFilter::clampAlpha(float & alpha)
{
	if (alpha > 1.0) { alpha = 1.0; }
	else if (alpha < 0.0) { alpha = 0.0; }
}

void LowPassFilter::getAlpha(float rate, float cutoff)
{
}

OneEuroFilter::OneEuroFilter()
{
}

OneEuroFilter::~OneEuroFilter()
{
}

void OneEuroFilter::filter()
{
}

void OneEuroFilter::lowPass(Point3f & output)
{
}
