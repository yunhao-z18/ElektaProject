#include "iterativeMethod.h"
#include <cmath>

double f(double delta, double A, double B, double Rx, double Ry, double lLR)
{
	double temp1, temp2;
	temp1 = A * sin(delta) - Rx;
	temp2 = B * cos(delta) - Ry;

	return (temp1 * temp1 + temp2 * temp2 - lLR * lLR);
}
//f(delta)=(A*sin(delta)-Rx)^2+(B*cos(delta)-Ry)^2-lLR^2

double df(double delta, double A, double B, double Rx, double Ry, double lLR)
{
	return ((A * A - B * B) * sin(2 * delta) - 2 * (A * Rx * cos(delta) - B * Ry * sin(delta)));
}

double solveDeltaIter(double A, double B, double Rx, double Ry, double lLR)
{
	double delta = 0;
	double delta_next = delta - f(delta, A, B, Rx, Ry, lLR) / df(delta, A, B, Rx, Ry, lLR);
	double error = 1e-6;
	int i = 0;

	while (abs(delta_next - delta) > error)
	{
		delta = delta_next;
		delta_next = delta - f(delta, A, B, Rx, Ry, lLR) / df(delta, A, B, Rx, Ry, lLR);
		i++;
	}

	return delta_next;
}