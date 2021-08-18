#include "cosineLaw.h"
#include <cmath>

//using radian measure 
double cosineLawGetAngle(double a, double b, double c)
{
	return acos((a * a + b * b - c * c) / (2 * a * b));
}

double cosineLawGetLength(double a, double b, double theta)
{
	return sqrt(a * a + b * b - 2 * a * b * cos(theta));
}