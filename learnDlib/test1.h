#pragma once

#include <dlib/matrix.h>

using namespace dlib;
using namespace std;

matrix<double, 4, 4> getF3_4_5_6(double X_xt, double Y_yt, double Z_vt, double psi_pr, double c_rad);
matrix<double, 3, 4> getD(matrix<double, 4, 4> F1_2, matrix<double, 4, 4> F7, double psi_pr, double c_rad, matrix<double, 4, 1> Vector1);
//¸Ä£¡£¡£¡
//double f_0(double theta_s, double theta_e, double psi_pr, double phi_rr, double Z_vt, double X_xt, double Y_yt, double phi, matrix<double, 4, 1> Vector1);
matrix<double, 3, 1> XYZ_0(double theta_s, double theta_e, double psi_pr, double phi_rr, double Z_vt, double X_xt, double Y_yt, matrix<double, 4, 1> Vector1);
