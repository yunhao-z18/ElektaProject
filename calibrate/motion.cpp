#include <iostream>
#include "motion.h"
#include "pitch1.h"
#include "roll1.h"
#include "calibrate.h"
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

#define PI 3.1415926

using namespace std;
using namespace Eigen;

const double PitchMotion::SP_DEFAULT = 250;
const double PitchMotion::AC_DEFAULT = 250;
const double PitchMotion::CN_DEFAULT = 432.7;
const double PitchMotion::PB_DEFAULT = 800;
const double PitchMotion::EH_DEFAULT = 655.8789;
const double PitchMotion::GE_DEFAULT = 800.3;
const double PitchMotion::GH_DEFAULT = 205.0325;
const double PitchMotion::GB_DEFAULT = 25;
const double PitchMotion::GF_DEFAULT = 119;
const double PitchMotion::FGB_DEFAULT = 165 * PI / 180;

VectorXd PitchMotion::getDefaultFixedVariables()
{
	VectorXd defaultFixedVariables(10);
	defaultFixedVariables << SP_DEFAULT, AC_DEFAULT, CN_DEFAULT, PB_DEFAULT, EH_DEFAULT, 
		GE_DEFAULT, GH_DEFAULT, GB_DEFAULT, GF_DEFAULT, FGB_DEFAULT;

	return defaultFixedVariables;
}

PitchMotion::PitchMotion(double lON):mLengthON(lON), mFixedVariables(getDefaultFixedVariables())
{}

PitchMotion::PitchMotion(double lON, VectorXd variables):mLengthON(lON), mFixedVariables(variables)
{}

void PitchMotion::setLengthON(double lON)
{
	mLengthON = lON;
}

double PitchMotion::getLengthON()const
{
	return mLengthON;
}

double PitchMotion::calcScrewLength(double angle)
{
	return pitchAngleToLength(mFixedVariables, angle, mLengthON);
}

double PitchMotion::calcAngle(double screwLength)
{
	return pitchLengthToAngle(mFixedVariables, screwLength, mLengthON);
}

struct PitchLmdifFunctor : Functor<double>
{
	PitchLmdifFunctor(int values, double LenON) :Functor<double>(10, values), mLenON(LenON){}
	VectorXd mAngles, mLengths;
	double mLenON;

	int operator()(const VectorXd& parameters, VectorXd& fvec) const
	{
		int i;
		for (i = 0; i < mAngles.size(); i++)
		{
			fvec(i) = mLengths(i) - pitchAngleToLength(parameters, mAngles(i), mLenON);
		}
		return 0;
	}
};

bool PitchMotion::calibrate(VectorXd& angles, VectorXd& lengths, int values, VectorXd& startParameters)
{
	PitchLmdifFunctor functor(values, mLengthON);
	functor.mAngles = angles;
	functor.mLengths = lengths;
	//y for test, displaying each fvec(i)
	VectorXd y(100);
	functor.operator()(startParameters, y);
	cout << "y first outpout: \n" << y << endl;

	NumericalDiff<PitchLmdifFunctor> numDiff(functor);
	LevenbergMarquardt<NumericalDiff<PitchLmdifFunctor> > lm(numDiff);
	lm.parameters.maxfev = 1000;
	int iRet = lm.minimize(startParameters);
	cout << "迭代次数 " << lm.iter << endl;

	functor.operator()(startParameters, y);
	cout << "y outpout(minimized): \n" << y << std::endl;

	return iRet;
}

void PitchMotion::displayCalibratedVariables(VectorXd& angles, VectorXd& lengths, int values, VectorXd& startParameters)
{
	if (calibrate(angles, lengths, values, startParameters))
	{
		cout << startParameters << endl;
	}
}

/***********************************************************************************************************/

const double RollMotion::IJ_DEFAULT = 286;
const double RollMotion::KL_DEFAULT = 5;
const double RollMotion::LR_DEFAULT = 20;
const double RollMotion::DN_DEFAULT = 38.49;
const double RollMotion::KN_DEFAULT = 5.05;
const double RollMotion::GAMMA_DEFAULT = PI / 6;

VectorXd RollMotion::getDefaultFixedVariables()
{
	VectorXd defaultFixedVariables(6);
	defaultFixedVariables << IJ_DEFAULT, KL_DEFAULT, LR_DEFAULT, DN_DEFAULT, KN_DEFAULT,
		GAMMA_DEFAULT;

	return defaultFixedVariables;
}

RollMotion::RollMotion():mFixedVariables(getDefaultFixedVariables())
{}

RollMotion::RollMotion(VectorXd variables):mFixedVariables(variables)
{}

double RollMotion::calcScrewLength(double angle)
{
	return rollAngleToLength(mFixedVariables, angle);
}

double RollMotion::calcAngle(double screwLength)
{
	return rollLengthToAngle(mFixedVariables, screwLength);
}

struct RollLmdifFunctor : Functor<double>
{
	RollLmdifFunctor(int values) :Functor<double>(6, values) {}
	VectorXd mAngles, mLengths;

	int operator()(const VectorXd& parameters, VectorXd& fvec) const
	{
		int i;
		for (i = 0; i < mAngles.size(); i++)
		{
			fvec(i) = mLengths(i) - rollAngleToLength(parameters, mAngles(i));
		}
		return 0;
	}
};

bool RollMotion::calibrate(VectorXd& angles, VectorXd& lengths, int values, VectorXd& startParameters)
{
	RollLmdifFunctor functor(values);
	functor.mAngles = angles;
	functor.mLengths = lengths;
	//y for test, displaying each fvec(i)
	VectorXd y(100);
	functor.operator()(startParameters, y);
	cout << "y first outpout: \n" << y << endl;

	NumericalDiff<RollLmdifFunctor> numDiff(functor);
	LevenbergMarquardt<NumericalDiff<RollLmdifFunctor> > lm(numDiff);
	lm.parameters.maxfev = 1000;
	int iRet = lm.minimize(startParameters);
	cout << "迭代次数 " << lm.iter << endl;

	functor.operator()(startParameters, y);
	cout << "y outpout(minimized): \n" << y << std::endl;

	return iRet;
}

void RollMotion::displayCalibratedVariables(VectorXd& angles, VectorXd& lengths, int values, VectorXd& startParameters)
{
	if (calibrate(angles, lengths, values, startParameters))
	{
		cout << startParameters << endl;
	}
}