#include "test1.h"
#include <math.h>
#include <dlib/matrix.h>

using namespace dlib;
using namespace std;

matrix<double, 4, 4> getF3_4_5_6(double X_xt, double Y_yt, double Z_vt, double psi_pr, double c_rad)
{
	matrix<double, 4, 4> M;
	M = 1, 0, 0, X_xt,
		0, cos(psi_pr + c_rad), -sin(psi_pr + c_rad), cos(psi_pr + c_rad)* (791 + Y_yt) - sin(psi_pr + c_rad) * 148.5 + 209,
		0, sin(psi_pr + c_rad), cos(psi_pr + c_rad), sin(psi_pr + c_rad)* (791 + Y_yt) + cos(psi_pr + c_rad) * 148.5 + Z_vt - 1079,
		0, 0, 0, 1;
	return M;
}

matrix<double, 3, 4> getD(matrix<double, 4, 4> F1_2, matrix<double, 4, 4> F7, double psi_pr, double c_rad, matrix<double, 4, 1> Vector1)
{
	matrix<double, 3, 4> D;
	matrix<double, 4, 1> Va, Vb, Vc, Vd;

	Vd = (F1_2 * (getF3_4_5_6(0, 0, 0, psi_pr, c_rad) * (F7 * Vector1)));
	Va = (F1_2 * (getF3_4_5_6(1, 0, 0, psi_pr, c_rad) * (F7 * Vector1))) - Vd;
	Vb = (F1_2 * (getF3_4_5_6(0, 1, 0, psi_pr, c_rad) * (F7 * Vector1))) - Vd;
	Vc = (F1_2 * (getF3_4_5_6(0, 0, 1, psi_pr, c_rad) * (F7 * Vector1))) - Vd;

	D = Va(0), Vb(0), Vc(0), Vd(0),
		Va(1), Vb(1), Vc(1), Vd(1),
		Va(2), Vb(2), Vc(2), Vd(2);

	return D;
}

matrix<double, 3, 1> XYZ_0(double theta_s, double theta_e, double psi_pr, double phi_rr, double Z_vt, double X_xt, double Y_yt, matrix<double, 4, 1> Vector1)
{
	double theta_s_e = theta_s + theta_e;
	matrix<double, 4, 4> F1_2, F3_4_5_6, F7;
	matrix<double, 4, 1> Vector2; //Vector2 = F1_2 * F3_4_5_6 * F7 * Vector1
	matrix<double, 3, 1> XYZ;

	F1_2 = cos(theta_s_e), -sin(theta_s_e), 0, 1000 * sin(theta_s),
		sin(theta_s_e), cos(theta_s_e), 0, -1000 * cos(theta_s),
		0, 0, 1, 0,
		0, 0, 0, 1;

	double element_03 = (sin(pi / 6 + phi_rr) - sin(pi / 6 - phi_rr)) * 286 / (4 * sin(pi / 6));
	double element_23 = 7.74 + (sin(pi / 6 + phi_rr) + sin(pi / 6 - phi_rr) - 2 * sin(pi / 6))
		* 286 / (4 * cos(pi / 6));

	F7 = cos(phi_rr), 0, sin(phi_rr), element_03,
		0, 1, 0, 0,
		-sin(phi_rr), 0, cos(phi_rr), element_23,
		0, 0, 0, 1;

	F3_4_5_6 = getF3_4_5_6(X_xt, Y_yt, Z_vt, psi_pr, 0);
	Vector2 = (F1_2 * (F3_4_5_6 * (F7 * Vector1)));
	XYZ(0) = Vector2(0);
	XYZ(1) = Vector2(1);
	XYZ(2) = Vector2(2);

	return XYZ;
}