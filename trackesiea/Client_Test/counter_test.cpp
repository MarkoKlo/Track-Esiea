#include "counter_test.h"

counter_test::counter_test() : current(0), last(0)
{
}


counter_test::~counter_test()
{
}

void counter_test::increment()
{
	current++;
	//printf("last:%d current:%d delta:%f\n", last, current);
	last = current;
}

