#pragma once
#include <Eigen\Dense>

using namespace Eigen;

double pitchAngleToLength(VectorXd FixedVariables, double alpha, double lON);
double pitchLengthToAngle(VectorXd FixedVariables, double lHF, double lON);