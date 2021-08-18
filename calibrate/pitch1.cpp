#include <cmath>
#include "cosineLaw.h"
#include "pitch1.h"
#define PI 3.1415926

//All angles in radius
double pitchAngleToLength(VectorXd FixedVariables, double alpha,double lON)
{
	double lSP = FixedVariables(0);
	double lAC = FixedVariables(1);
	double lCN = FixedVariables(2);
	double lPB = FixedVariables(3);
	double lEH = FixedVariables(4);
	double lGE = FixedVariables(5);
	double lGH = FixedVariables(6);
	double lGB = FixedVariables(7);
	double lGF = FixedVariables(8);
	double aFGB = FixedVariables(9);
	double lHF;

	double lSE = lSP;
	double lBD = lAC;
	double lCD = lPB;
	double lOD = lCD - lCN;
	double lND = lOD;
	double lBF = cosineLawGetLength(lGB, lGF, aFGB);
	double aEGH = cosineLawGetAngle(lGE, lGH, lEH);

	double aNOD = cosineLawGetAngle(lON, lOD, lND);
	double aAPB = aNOD;
	double lPE = cosineLawGetLength(lSP, lSE, alpha);

	double aEPB;
	if (alpha < 0)
	{
		aEPB = 3 * PI / 2 - aAPB + alpha / 2;
	}
	else
	{
		aEPB = PI / 2 - aAPB + alpha / 2;
	}

	double lEB = cosineLawGetLength(lPE, lPB, aEPB);
	double aEGB = cosineLawGetAngle(lGE, lGB, lEB);
	double aHGF = 2 * PI - aEGB - aFGB - aEGH;

	lHF = cosineLawGetLength(lGH, lGF, aHGF);

	return lHF;
}

double pitchLengthToAngle(VectorXd FixedVariables, double lHF, double lON)
{
	double lSP = FixedVariables(0);
	double lAC = FixedVariables(1);
	double lCN = FixedVariables(2);
	double lPB = FixedVariables(3);
	double lEH = FixedVariables(4);
	double lGE = FixedVariables(5);
	double lGH = FixedVariables(6);
	double lGB = FixedVariables(7);
	double lGF = FixedVariables(8);
	double aFGB = FixedVariables(9);
	double alpha;

	double lSE = lSP;
	double lBD = lAC;
	double lCD = lPB;
	double lOD = lCD - lCN;
	double lND = lOD;
	double lBF = cosineLawGetLength(lGB, lGF, aFGB);
	double aEGH = cosineLawGetAngle(lGE, lGH, lEH);

	double aHGF = cosineLawGetAngle(lGF, lGH, lHF);
	double aEGB = 2 * PI - aHGF - aFGB - aEGH;
	double lEB = cosineLawGetLength(lGE, lGB, aEGB);
	double aNOD = cosineLawGetAngle(lON, lOD, lND);
	double aAPB = aNOD;
	double aSPB = PI - aAPB;
	double lBS = cosineLawGetLength(lSP, lPB, aSPB);
	double aPSB = asin(lPB * sin(aSPB) / lBS);
	double aESB = cosineLawGetAngle(lSE, lBS, lEB);

	alpha = aPSB - aESB;

	return alpha;
}