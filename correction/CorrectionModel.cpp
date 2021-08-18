#include "Transformation.h"
#include "CorrectionModel.h"
#include "GenericFunctor.h"
#include <Eigen\Dense>
#include <unsupported/Eigen/NonLinearOptimization>

CorrectionModel::CorrectionModel(Iso& iso, Column& col, VerticalTranslation& vt, Pitch& pitch,
	LateralTranslation& xt, LongitudeTranslation& yt, Roll& roll, TableTop& tt) :
	mIso(iso), mColumn(col), mVerticalTranslation(vt), mPitch(pitch),
	mLateralTranslation(xt), mLongitudeTranslation(yt), mRoll(roll), mTableTop(tt){}

struct LmFunctor : Functor<double>
{
    LmFunctor(void) : Functor<double>(6, 12) {}

    int operator()(const Eigen::VectorXd& inputs, Eigen::VectorXd& fvec)
    {

        return 0;
    }
    int df(const Eigen::VectorXd& inputs, Eigen::MatrixXd& fjac)
    {

        return 0;
    }
};


bool CorrectionModel::correction(Eigen::Matrix4d correctionMatrix)
{
    int info;
    VectorXd start(6);
    LmFunctor functor;
    LevenbergMarquardt<LmFunctor> lm(functor);
    info = lm.minimize(start);


    return info;
}