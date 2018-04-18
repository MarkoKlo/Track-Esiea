#pragma once
#include<stdio.h>
#include<stdlib.h>
#include<opencv2\opencv.hpp>

class counter_test
{
public:
	counter_test::counter_test();
	counter_test::~counter_test();
	void track();
	void get_delta_time();

private:
	int m_currentTick;
	int m_lastTick;
	float m_deltaTime;
	int64 m_current;
	int64 m_last;
};

