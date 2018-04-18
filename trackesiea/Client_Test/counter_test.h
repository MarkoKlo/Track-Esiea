#pragma once
#include<stdio.h>
#include<stdlib.h>

class counter_test
{
public:
	counter_test();
	~counter_test();
	void increment();
private:
	int last;
	int current;
};

