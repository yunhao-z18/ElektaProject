#include "transformation.h"
#include <cmath>

#define PI 3.14159265

Iso::Iso(double thetaS): mThetaS(thetaS){}
Eigen::Matrix4d Iso::getTransformationMatrix()const
{
	Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
	transformationMatrix(0, 0) = cos(mThetaS);
	transformationMatrix(0, 1) = -sin(mThetaS);
	transformationMatrix(1, 0) = sin(mThetaS);
	transformationMatrix(1, 1) = cos(mThetaS);

	return transformationMatrix;
}
void Iso::transformVector(Eigen::Vector4d& vec)
{
	vec = Iso::getTransformationMatrix() * vec;
}

const double Column::lEDefault = -1000;
Column::Column(double thetaE, double lE): mThetaE(thetaE), mLE(lE){}
Eigen::Matrix4d Column::getTransformationMatrix()const
{
	Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
	transformationMatrix(0, 0) = cos(mThetaE);
	transformationMatrix(0, 1) = -sin(mThetaE);
	transformationMatrix(1, 0) = sin(mThetaE);
	transformationMatrix(1, 1) = cos(mThetaE);
	transformationMatrix(1, 3) = mLE;

	return transformationMatrix;
}
void Column::transformVector(Eigen::Vector4d& vec)
{
	vec = Column::getTransformationMatrix() * vec;
}

const double VerticalTranslation::lVtYDefault = 209;
const double VerticalTranslation::lVtZDefault = -1079;
VerticalTranslation::VerticalTranslation(double lVtY, double lVtZ): mLVtY(lVtY), mLVtZ(lVtZ){}
Eigen::Matrix4d VerticalTranslation::getTransformationMatrix()const
{
	Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
	transformationMatrix(1, 3) = mLVtY;
	transformationMatrix(2, 3) = mLVtZ;

	return transformationMatrix;
}
void VerticalTranslation::transformVector(Eigen::Vector4d& vec)
{
	vec = VerticalTranslation::getTransformationMatrix() * vec;
}

Pitch::Pitch(double psiPr, double zVt): mPsiPr(psiPr), mZVt(zVt){}
Eigen::Matrix4d Pitch::getTransformationMatrix()const
{
	Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
	transformationMatrix(1, 1) = cos(mPsiPr);
	transformationMatrix(1, 2) = -sin(mPsiPr);
	transformationMatrix(2, 1) = sin(mPsiPr);
	transformationMatrix(2, 2) = cos(mPsiPr);
	transformationMatrix(2, 3) = mZVt;

	return transformationMatrix;
}
void Pitch::transformVector(Eigen::Vector4d& vec)
{
	vec = Pitch::getTransformationMatrix() * vec;
}

const double LateralTranslation::lXtDefault = 40.5;
LateralTranslation::LateralTranslation(double xXt, double lXt): mXXt(xXt), mLXt(lXt){}
Eigen::Matrix4d LateralTranslation::getTransformationMatrix()const
{
	Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
	transformationMatrix(0, 3) = mXXt;
	transformationMatrix(2, 3) = mLXt;

	return transformationMatrix;
}
void LateralTranslation::transformVector(Eigen::Vector4d& vec)
{
	vec = LateralTranslation::getTransformationMatrix() * vec;
}

const double LongitudeTranslation::lYtYDefault = 791;
const double LongitudeTranslation::lYtZDefault = 108;
LongitudeTranslation::LongitudeTranslation(double yYt, double lYtY, double lYtZ): mYYt(yYt), mLYtY(lYtY), mLYtZ(lYtZ){}
Eigen::Matrix4d LongitudeTranslation::getTransformationMatrix()const
{
	Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
	transformationMatrix(1, 3) = mYYt + mLYtY;
	transformationMatrix(2, 3) = mLYtZ;

	return transformationMatrix;
}
void LongitudeTranslation::transformVector(Eigen::Vector4d& vec)
{
	vec = LongitudeTranslation::getTransformationMatrix() * vec;
}

const double Roll::gammaRrDefault = PI / 60;
const double Roll::lIjDefault = 286;
const double Roll::lRrZDefault = 7.74;
Roll::Roll(double phiRr, double gammaRr, double lIj, double lRrZ): mPhiRr(phiRr), mGammaRr(gammaRr), mLIj(lIj), mLRrZ(lRrZ){}
Eigen::Matrix4d Roll::getTransformationMatrix()const
{
	Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
	transformationMatrix(0, 0) = cos(mPhiRr);
	transformationMatrix(0, 2) = sin(mPhiRr);
	transformationMatrix(2, 0) = -sin(mPhiRr);
	transformationMatrix(2, 2) = cos(mPhiRr);
	transformationMatrix(0, 3) = (sin(mGammaRr + mPhiRr) - sin(mGammaRr - mPhiRr)) * mLIj / (4 * sin(mGammaRr));
	transformationMatrix(2, 3) = 
		(sin(mGammaRr + mPhiRr) + sin(mGammaRr - mPhiRr) - 2 * sin(mGammaRr)) * mLIj / (4 * cos(mGammaRr)) + mLRrZ;

	return transformationMatrix;
}
void Roll::transformVector(Eigen::Vector4d& vec)
{
	vec = Roll::getTransformationMatrix() * vec;
}

const double TableTop::lTZDefault = 82;
TableTop::TableTop(double lTZ): mLTZ(lTZ){}
Eigen::Matrix4d TableTop::getTransformationMatrix()const
{
	Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
	transformationMatrix(2, 3) = mLTZ;

	return transformationMatrix;
}
void TableTop::transformVector(Eigen::Vector4d& vec)
{
	vec = TableTop::getTransformationMatrix() * vec;
}

Eigen::Matrix4d finalTransformationMatrix(Iso iso, Column col, VerticalTranslation vt, Pitch pitch,
	LateralTranslation xt, LongitudeTranslation yt, Roll roll, TableTop tt)
{
	Eigen::Matrix4d transformationMatrix;
	transformationMatrix = iso.getTransformationMatrix() * col.getTransformationMatrix() * 
		vt.getTransformationMatrix() * pitch.getTransformationMatrix() * 
		xt.getTransformationMatrix() * yt.getTransformationMatrix() * 
		roll.getTransformationMatrix() * tt.getTransformationMatrix();

	return transformationMatrix;
}

void tableTopToIso(Eigen::Vector4d& vec, Iso iso, Column col, VerticalTranslation vt, Pitch pitch,
	LateralTranslation xt, LongitudeTranslation yt, Roll roll, TableTop tt)
{
	vec = finalTransformationMatrix(iso, col, vt, pitch, xt, yt, roll, tt) * vec;
}