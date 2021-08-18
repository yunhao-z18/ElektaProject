#pragma once

#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

//三元二次多项式求最值
//param = [a1,b1,d1,a2,b2,d2,b3,d3]
double f_XYZ(VectorXd param, Vector3d XYZ);
Vector3d grad_f_XYZ(VectorXd param, Vector3d XYZ);
Vector3d XYZ_critical(VectorXd param);
bool is_inside_3D(Vector3d XYZ, double x1, double x2, double y1, double y2, double z1, double z2);
Vector3d XYZ_max_boundary_3D(VectorXd param, double x1, double x2, double y1, double y2, double z1, double z2);
Vector3d XYZ_max(VectorXd param, double x1, double x2, double y1, double y2, double z1, double z2);
double f_XYZ_max(VectorXd param, double x1, double x2, double y1, double y2, double z1, double z2);
//最值落在边界处，转化为二元函数

//二元二次多项式求最值(注意param的内容变了)
//param = [A,B,C,D,E,F]
double f_XY(VectorXd param, double x, double y);
Vector2d grad_f_XY(VectorXd param, double x, double y);
Vector2d XY_critical(VectorXd param);
bool is_inside_2D(Vector2d XY, double x1, double x2, double y1, double y2);
Vector2d XY_max_boundary_2D(VectorXd param, double x1, double x2, double y1, double y2);
Vector2d XY_max(VectorXd param, double x1, double x2, double y1, double y2);
double f_XY_max(VectorXd param, double x1, double x2, double y1, double y2);
//最值落在边界处，转化为一元函数（抛物线）

//抛物线求最大值点
double X_max(double a, double b, double x1, double x2);