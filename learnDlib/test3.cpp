#include <iostream>
#include <Eigen/Dense>
#include "test3.h"

using namespace std;
using namespace Eigen;

//三元二次多项式求最值
//param = [a1,b1,d1,a2,b2,d2,b3,d3]
double f_XYZ(VectorXd param, Vector3d XYZ)
{
	double x1,y1,z1;
	x1 = param[0] * XYZ[0] + param[1] * XYZ[1] + param[2];
	y1 = param[3] * XYZ[0] + param[4] * XYZ[1] + param[5];
	z1 = param[6] * XYZ[1] + XYZ[2] + param[7];
	
	return pow(x1, 2) + pow(y1, 2) + pow(z1, 2);
}

Vector3d grad_f_XYZ(VectorXd param, Vector3d XYZ)
{
	Vector3d grad;
	double x1, y1, z1;

	x1 = param[0] * XYZ[0] + param[1] * XYZ[1] + param[2];
	y1 = param[3] * XYZ[0] + param[4] * XYZ[1] + param[5];
	z1 = param[6] * XYZ[1] + XYZ[2] + param[7];
	grad[0] = 2 * (param[0] * x1 + param[3] * y1);
	grad[1] = 2 * (param[1] * x1 + param[4] * y1 + param[7] * z1);
	grad[2] = 2 * z1;

	return grad;
}

Vector3d XYZ_critical(VectorXd param)
{
	Matrix3d A;
	Vector3d b;
	A << param[0] * param[0]+ param[3] * param[3], param[0] * param[1]+ param[3] * param[4], 0,
		param[0] * param[1]+ param[3] * param[4], param[1] * param[1]+ param[4] * param[4] + param[6] * param[6], param[6],
		0, param[6], 1;
	b << -param[0]*param[2]- param[3]*param[5], -param[1]* param[2]- param[4]* param[5]- param[6]* param[7], -param[7];

	return A.colPivHouseholderQr().solve(b);
}

bool is_inside_3D(Vector3d XYZ, double x1, double x2, double y1, double y2, double z1, double z2)
{
	return (XYZ[0] > x1 && XYZ[0] < x2 && XYZ[1] > y1 && XYZ[1] < y2 && XYZ[2] > z1 && XYZ[2] < z2);
}

Vector3d XYZ_max_boundary_3D(VectorXd param, double x1, double x2, double y1, double y2, double z1, double z2)
{
	VectorXd param_2D = VectorXd::Zero(6);
	Vector3d XYZ_boundary;
	Vector3d XYZ_temp;
	double f_XYZ_boundary;

	//x=x1
	param_2D[0] = param[1] * param[1] + param[4] * param[4] + param[6] * param[6];
	param_2D[1] = 1;
	param_2D[2] = 2 * param[6];
	param_2D[3] = 2 * param[1] * (param[0] * x1 + param[2])+ 2 * param[4] * (param[3] * x1 + param[5])+ 2 * param[6] * param[7];
	param_2D[4] = 2 * param[7];
	param_2D[5] = 0;

	XYZ_temp[0] = x1;
	XYZ_temp[1] = XY_max(param_2D, y1, y2, z1, z2)[0];
	XYZ_temp[2] = XY_max(param_2D, y1, y2, z1, z2)[1];
	f_XYZ_boundary = f_XYZ(param, XYZ_temp);
	XYZ_boundary = XYZ_temp;

	//x=x2
	param_2D[3] = 2 * param[1] * (param[0] * x2 + param[2]) + 2 * param[4] * (param[3] * x2 + param[5]) + 2 * param[6] * param[7];

	XYZ_temp[0] = x2;
	XYZ_temp[1] = XY_max(param_2D, y1, y2, z1, z2)[0];
	XYZ_temp[2] = XY_max(param_2D, y1, y2, z1, z2)[1];
	if (f_XYZ(param, XYZ_temp) > f_XYZ_boundary)
	{
		f_XYZ_boundary = f_XYZ(param, XYZ_temp);
		XYZ_boundary = XYZ_temp;
	}
	
	//y=y1
	param_2D[0] = param[0] * param[0] + param[3] * param[3];
	param_2D[2] = 0;
	param_2D[3] = 2 * param[0] * (param[1] * y1 + param[2]) + 2 * param[3] * (param[4] * y1 + param[5]);
	param_2D[4] = 2 * (param[6] * y1 + param[7]);

	XYZ_temp[0] = XY_max(param_2D, x1, x2, z1, z2)[0];
	XYZ_temp[1] = y1;
	XYZ_temp[2] = XY_max(param_2D, x1, x2, z1, z2)[1];
	if (f_XYZ(param, XYZ_temp) > f_XYZ_boundary)
	{
		f_XYZ_boundary = f_XYZ(param, XYZ_temp);
		XYZ_boundary = XYZ_temp;
	}

	//y=y2
	param_2D[3] = 2 * param[0] * (param[1] * y2 + param[2]) + 2 * param[3] * (param[4] * y2 + param[5]);
	param_2D[4] = 2 * (param[6] * y2 + param[7]);

	XYZ_temp[0] = XY_max(param_2D, x1, x2, z1, z2)[0];
	XYZ_temp[1] = y2;
	XYZ_temp[2] = XY_max(param_2D, x1, x2, z1, z2)[1];
	if (f_XYZ(param, XYZ_temp) > f_XYZ_boundary)
	{
		f_XYZ_boundary = f_XYZ(param, XYZ_temp);
		XYZ_boundary = XYZ_temp;
	}

	//z=z1
	param_2D[1] = param[1] * param[1] + param[4] * param[4] + param[6] * param[6];
	param_2D[2] = 2 * param[0] * param[1] + 2 * param[3] * param[4];
	param_2D[3] = 2 * param[0] * param[2] + 2 * param[3] * param[5];
	param_2D[4] = 2 * param[1] * param[2] + 2 * param[4] * param[5] + 2 * param[6] * (param[7] + z1);

	XYZ_temp[0] = XY_max(param_2D, x1, x2, y1, y2)[0];
	XYZ_temp[1] = XY_max(param_2D, x1, x2, y1, y2)[1];
	XYZ_temp[2] = z1;
	if (f_XYZ(param, XYZ_temp) > f_XYZ_boundary)
	{
		f_XYZ_boundary = f_XYZ(param, XYZ_temp);
		XYZ_boundary = XYZ_temp;
	}

	//z=z2
	param_2D[4] = 2 * param[1] * param[2] + 2 * param[4] * param[5] + 2 * param[6] * (param[7] + z2);

	XYZ_temp[0] = XY_max(param_2D, x1, x2, y1, y2)[0];
	XYZ_temp[1] = XY_max(param_2D, x1, x2, y1, y2)[1];
	XYZ_temp[2] = z2;
	if (f_XYZ(param, XYZ_temp) > f_XYZ_boundary)
	{
		f_XYZ_boundary = f_XYZ(param, XYZ_temp);
		XYZ_boundary = XYZ_temp;
	}

	return XYZ_boundary;
}

Vector3d XYZ_max(VectorXd param, double x1, double x2, double y1, double y2, double z1, double z2)
{
	Vector3d XYZ1, XYZ2;
	XYZ1 = XYZ_max_boundary_3D(param, x1, x2, y1, y2, z1, z2);
	XYZ2 = XYZ_critical(param);

	if (f_XYZ(param, XYZ1) >= f_XYZ(param, XYZ2))
	{
		return XYZ1;
	}
	else
	{
		return XYZ2;
	}
}

double f_XYZ_max(VectorXd param, double x1, double x2, double y1, double y2, double z1, double z2)
{
	return f_XYZ(param, XYZ_max(param, x1, x2, y1, y2, z1, z2));
}
//最值落在边界处，转化为二元函数

//二元二次多项式求最值(注意param的内容变了)
//param = [A,B,C,D,E,F]
double f_XY(VectorXd param, double x, double y)
{
	return (param[0] * x * x + param[1] * y * y + param[2] * x * y + param[3] * x + param[4] * y + param[5]);
}

Vector2d grad_f_XY(VectorXd param, double x, double y)
{
	Vector2d grad;
	
	grad[0] = 2 * param[0] * x + param[2] * y + param[3];
	grad[1] = 2 * param[1] * y + param[2] * x + param[4];

	return grad;
}

Vector2d XY_critical(VectorXd param)
{
	Matrix2d A;
	Vector2d b;
	A << 2 * param[0], param[2],
		param[2], 2 * param[1];
	b << -param[3], -param[4];

	return A.colPivHouseholderQr().solve(b);
}

bool is_inside_2D(Vector2d XY, double x1, double x2, double y1, double y2)
{
	return (XY[0] > x1 && XY[0] < x2 && XY[1] > y1 && XY[1] < y2);
}

Vector2d XY_max_boundary_2D(VectorXd param, double x1, double x2, double y1, double y2)
{
	double a, b;
	Vector2d XY_boundary;
	double f_XY_boundary;

	//x=x1
	a = param[1];
	b = param[2] * x1;
	f_XY_boundary = f_XY(param, x1, X_max(a, b, y1, y2));
	XY_boundary[0] = x1;
	XY_boundary[1] = X_max(a, b, y1, y2);

	//x=x2
	b = param[2] * x2;
	if (f_XY(param, x2, X_max(a, b, y1, y2)) > f_XY_boundary)
	{
		f_XY_boundary = f_XY(param, x2, X_max(a, b, y1, y2));
		XY_boundary[0] = x2;
		XY_boundary[1] = X_max(a, b, y1, y2);
	}

	//y=y1
	a = param[0];
	b = param[2] * y1;
	if (f_XY(param, X_max(a, b, x1, x2), y1) > f_XY_boundary)
	{
		f_XY_boundary = f_XY(param, X_max(a, b, x1, x2), y1);
		XY_boundary[0] = X_max(a, b, x1, x2);
		XY_boundary[1] = y1;
	}

	//y=y2
	b = param[2] * y2;
	if (f_XY(param, X_max(a, b, x1, x2), y2) > f_XY_boundary)
	{
		f_XY_boundary = f_XY(param, X_max(a, b, x1, x2), y2);
		XY_boundary[0] = X_max(a, b, x1, x2);
		XY_boundary[1] = y2;
	}

	return XY_boundary;
}

Vector2d XY_max(VectorXd param, double x1, double x2, double y1, double y2)
{
	Vector2d XY;

	if (is_inside_2D(XY, x1, x2, y1, y2))
	{
		XY = XY_critical(param);
	}
	else
	{
		XY = XY_max_boundary_2D(param, x1, x2, y1, y2);
	}

	return XY;
}

double f_XY_max(VectorXd param, double x1, double x2, double y1, double y2)
{
	return f_XY(param, XY_max(param, x1, x2, y1, y2)[0], XY_max(param, x1, x2, y1, y2)[1]);
}
//最值落在边界处，转化为一元二次（抛物线）

//抛物线求最大值点
double X_max(double a, double b, double x1, double x2)
{
	double p_x1 = a * x1 * x1 + b * x1;
	double p_x2 = a * x2 * x2 + b * x2;
	double larger_x = (p_x1 > p_x2) ? x1 : x2;

	if (a >= 0) { return larger_x; }
	else
	{
		double x0 = -0.5 * b / a;
		if (x0 > x1 && x0 < x2) { return x0; }
		else { return larger_x; }
	}
}



//int main()
//{
//	VectorXd param = VectorXd::Zero(8);
//	param << 1, 2, 3, 4, 5, 6, 7, 8;
//	Vector3d critical = XYZ_critical(param);
//	Vector3d grad = grad_f_XYZ(param, critical);
//
//	//cout << param << endl;
//	cout << critical << endl;
//	cout << grad << endl;
//
//	cout << is_inside_3D(critical, 0, 2, -4, 4, -10, 10) << endl;
//	Vector3d XYZ = XYZ_max(param, 0, 2, -4, 4, -10, 10);
//	cout << XYZ << endl;
//	return 0;
//}



