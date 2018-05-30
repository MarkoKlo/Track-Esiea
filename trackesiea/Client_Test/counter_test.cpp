#include "counter_test.h"


counter_test::counter_test() : m_current(0), m_last(0)
{
}


counter_test::~counter_test()
{
}

void counter_test::track()
{
		get_delta_time(); //printf("delta:%f\n", m_deltaTime);
}

void counter_test::get_delta_time()
{
	m_current = cv::getTickCount();
	//printf("last:%I64d current:%I64d delta:%I64d\n", m_last, m_current, m_last - m_current);
	m_last = m_current;

}

