#pragma once
#include <Eigen\Dense>

using namespace Eigen;

double rollAngleToLength(VectorXd FixedVariables, double delta);
double rollLengthToAngle(VectorXd FixedVariables, double lDR);