#pragma once
#include <Eigen\Dense>

using namespace Eigen;

class Motion
{
public:
	virtual double calcScrewLength(double angle) = 0;
	virtual double calcAngle(double screwLength) = 0;
	virtual bool calibrate(VectorXd& angles, VectorXd& lengths, int values, VectorXd& startParameters) = 0;
	virtual void displayCalibratedVariables(VectorXd& angles, VectorXd& lengths, int values, VectorXd& startParameters) = 0;
	virtual ~Motion() {}
};

class PitchMotion :public Motion
{
public:
	PitchMotion(double lON);
	PitchMotion(double lON, VectorXd variables);
	static VectorXd getDefaultFixedVariables();
	void setLengthON(double lON);
	double getLengthON()const;

	virtual double calcScrewLength(double angle);
	virtual double calcAngle(double screwlength);
	virtual bool calibrate(VectorXd& angles, VectorXd& lengths, int values, VectorXd& startParameters);
	virtual void displayCalibratedVariables(VectorXd& angles, VectorXd& lengths, int values, VectorXd& startParameters);

private:
	double mLengthON;
	static const double SP_DEFAULT;
	static const double AC_DEFAULT;
	static const double CN_DEFAULT;
	static const double PB_DEFAULT;
	static const double EH_DEFAULT;
	static const double GE_DEFAULT;
	static const double GH_DEFAULT;
	static const double GB_DEFAULT;
	static const double GF_DEFAULT;
	static const double FGB_DEFAULT;

	VectorXd mFixedVariables;
};

class RollMotion :public Motion
{
public:
	RollMotion();
	RollMotion(VectorXd variables);
	static VectorXd getDefaultFixedVariables();

	virtual double calcScrewLength(double angle);
	virtual double calcAngle(double screwlength);
	virtual bool calibrate(VectorXd& angles, VectorXd& lengths, int values, VectorXd& startParameters);
	virtual void displayCalibratedVariables(VectorXd& angles, VectorXd& lengths, int values, VectorXd& startParameters);

private:
	static const double IJ_DEFAULT;
	static const double KL_DEFAULT;
	static const double LR_DEFAULT;
	static const double DN_DEFAULT;
	static const double KN_DEFAULT;
	static const double GAMMA_DEFAULT;

	VectorXd mFixedVariables;
};
