#include <cmath>
#include "cosineLaw.h"
#include "roll1.h"
#include "iterativeMethod.h"
#define PI 3.1415926

//All angles in radius
double rollAngleToLength(VectorXd FixedVariables, double delta)
{
	double lIJ = FixedVariables(0);
	double lKL = FixedVariables(1);
	double lLR = FixedVariables(2);
	double lDN = FixedVariables(3);
	double lKN = FixedVariables(4);
	double gamma = FixedVariables(5);

	double Ny = lIJ * tan(gamma) / 2 - lKN;
	double Kx = lIJ * sin(delta) / (2 * tan(gamma));
	double Ky = lIJ * tan(gamma) * cos(delta) / 2;
	double Lx = Kx + lKL * sin(delta);
	double Ly = Ky - lKL * cos(delta);
	double lER = sqrt(lLR * lLR - (Ny - Ly) * (Ny - Ly));
	double lDR = lDN + Lx + lER;

	return lDR;
}

double rollLengthToAngle(VectorXd FixedVariables, double lDR)
{
	double lIJ = FixedVariables(0);
	double lKL = FixedVariables(1);
	double lLR = FixedVariables(2);
	double lDN = FixedVariables(3);
	double lKN = FixedVariables(4);
	double gamma = FixedVariables(5);

	double Rx = lDR - lDN;
	double Ry = lIJ * tan(gamma) / 2 - lKN;
	//Lx=A*sin(delta),Ly=B*cos(delta)
	double A = lIJ / (2 * tan(gamma)) + lKL;
	double B = lIJ * tan(gamma) / 2 - lKL;
	double delta = solveDeltaIter(A, B, Rx, Ry, lLR);

	return delta;
}