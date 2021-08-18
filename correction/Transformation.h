#pragma once
#include <Eigen\Dense>

class Transformation
{
public:
	virtual Eigen::Matrix4d getTransformationMatrix()const = 0;
	virtual void transformVector(Eigen::Vector4d& vec) = 0;
	~Transformation(){}
};

class Iso : public Transformation
{
public:
	Iso(double thetaS);
	Eigen::Matrix4d getTransformationMatrix()const;
	void transformVector(Eigen::Vector4d& vec);
private:
	double mThetaS;	
};

class Column : public Transformation
{
public:
	static const double lEDefault;
	Column(double thetaE, double lenE);
	Eigen::Matrix4d getTransformationMatrix()const;
	void transformVector(Eigen::Vector4d& vec);

private:
	double mLE;
	double mThetaE;
};

class VerticalTranslation : public Transformation
{
public:
	static const double lVtYDefault, lVtZDefault;
	VerticalTranslation(double lVtY, double lVtZ);
	Eigen::Matrix4d getTransformationMatrix()const;
	void transformVector(Eigen::Vector4d& vec);

private:
	double mLVtY;
	double mLVtZ;
};

class Pitch : public Transformation
{
public:
	Pitch(double psiPr, double zVt);
	Eigen::Matrix4d getTransformationMatrix()const;
	void transformVector(Eigen::Vector4d& vec);

private:
	double mPsiPr;
	double mZVt;
};

class LateralTranslation : public Transformation
{
public:
	static const double lXtDefault;
	LateralTranslation(double xXt, double lXt);
	Eigen::Matrix4d getTransformationMatrix()const;
	void transformVector(Eigen::Vector4d& vec);

private:
	double mXXt;
	double mLXt;
};

class LongitudeTranslation : public Transformation
{
public:
	static const double lYtYDefault, lYtZDefault;
	LongitudeTranslation(double yYt, double lYtY, double lYtZ);
	Eigen::Matrix4d getTransformationMatrix()const;
	void transformVector(Eigen::Vector4d& vec);

private:
	double mYYt;
	double mLYtY;
	double mLYtZ;
};

class Roll : public Transformation
{
public:
	static const double gammaRrDefault, lIjDefault, lRrZDefault;
	Roll(double phiRr, double gammaRr, double lIj, double lRrZ);
	Eigen::Matrix4d getTransformationMatrix()const;
	void transformVector(Eigen::Vector4d& vec);

private:
	double mPhiRr;
	double mGammaRr;
	double mLIj;
	double mLRrZ;
};

class TableTop : public Transformation
{
public:
	static const double lTZDefault;
	TableTop(double lTZ);
	Eigen::Matrix4d getTransformationMatrix()const;
	void transformVector(Eigen::Vector4d& vec);

private:
	double mLTZ;
};

Eigen::Matrix4d finalTransformationMatrix(Iso iso, Column col, VerticalTranslation vt, Pitch pitch,
	LateralTranslation xt, LongitudeTranslation yt, Roll roll, TableTop tt);
void tableTopToIso(Eigen::Vector4d& vec, Iso iso, Column col, VerticalTranslation vt, Pitch pitch,
	LateralTranslation xt, LongitudeTranslation yt, Roll roll, TableTop tt);