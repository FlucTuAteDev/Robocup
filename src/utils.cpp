#include "utils.h"

double clamp(double d, double min, double max)
{
	const double t = d < min ? min : d;
	return t > max ? max : t;
}

int sign(int num) {
	return (num > 0) - (num < 0);
}