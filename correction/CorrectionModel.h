#pragma once
#include <Eigen\Dense>
#include "Transformation.h"

class CorrectionModel
{
public:
	CorrectionModel(Iso& iso, Column& col, VerticalTranslation& vt, Pitch& pitch,
		LateralTranslation& xt, LongitudeTranslation& yt, Roll& roll, TableTop& tt);
	bool correction(Eigen::Matrix4d correctionMatrix);

	Iso mIso;
	Column mColumn;
	VerticalTranslation mVerticalTranslation;
	Pitch mPitch;
	LateralTranslation mLateralTranslation;
	LongitudeTranslation mLongitudeTranslation;
	Roll mRoll;
	TableTop mTableTop;
};