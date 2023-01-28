#include "main.h"
#include "filter.h"
#include "filter_task.h"

extern unsigned int volt[2];
MeanFilter<5> MDF0,MDF1;

int filter_task(){
	volt[0] = MDF0.f(volt[0]);
	volt[1] = MDF1.f(volt[1]);
	return 0;
}
