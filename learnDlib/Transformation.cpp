#include <iostream>
#include <math.h>
#include <dlib/matrix.h>

using namespace dlib;
using namespace std;

class Transformation
{
public:
	Transformation(){}
	virtual ~Transformation(){}
	virtual matrix<double, 4, 4> getMatrix() const= 0;

};

class F_s : public Transformation
{
public:
	F_s(double theta_s):m_theta_s(theta_s){}
	~F_s(){}
	matrix<double, 4, 4> getMatrix() const
	{
		matrix<double, 4, 4> M;

		M = cos(m_theta_s), -sin(m_theta_s), 0, 0,
			sin(m_theta_s), cos(m_theta_s), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

		return M;
	}

private:
	double m_theta_s;
};

class F_e : public Transformation
{
public:
	F_e(double theta_e, double L_e) :m_theta_e(theta_e), m_L_e(L_e) {}
	~F_e() {}
	matrix<double, 4, 4> getMatrix() const
	{
		matrix<double, 4, 4> M;

		M = cos(m_theta_e), -sin(m_theta_e), 0, 0,
			sin(m_theta_e), cos(m_theta_e), 0, m_L_e,
			0, 0, 1, 0,
			0, 0, 0, 1;

		return M;
	}

private:
	double m_theta_e;
	double m_L_e;
};

class F_vt : public Transformation
{
public:
	F_vt(double L_vty, double L_vtz) :m_L_vty(L_vty), m_L_vtz(L_vtz) {}
	~F_vt() {}
	matrix<double, 4, 4> getMatrix() const
	{
		matrix<double, 4, 4> M;

		M = 1, 0, 0, 0,
			0, 1, 0, m_L_vty,
			0, 0, 1, m_L_vtz,
			0, 0, 0, 1;

		return M;
	}

private:
	double m_L_vty;
	double m_L_vtz;
};

class F_pr : public Transformation
{
public:
	F_pr(double psi_pr, double Z_vt) :m_psi_pr(psi_pr), m_Z_vt(Z_vt) {}
	~F_pr() {}
	matrix<double, 4, 4> getMatrix() const
	{
		matrix<double, 4, 4> M;

		M = 1, 0, 0, 0,
			0, cos(m_psi_pr), -sin(m_psi_pr), 0,
			0, sin(m_psi_pr), cos(m_psi_pr), m_Z_vt,
			0, 0, 0, 1;

		return M;
	}

private:
	double m_psi_pr;
	double m_Z_vt;
};

class F_xt : public Transformation
{
public:
	F_xt(double X_xt, double L_xt) :m_X_xt(X_xt), m_L_xt(L_xt) {}
	~F_xt() {}
	matrix<double, 4, 4> getMatrix() const
	{
		matrix<double, 4, 4> M;

		M = 1, 0, 0, m_X_xt,
			0, 1, 0, 0,
			0, 0, 1, m_L_xt,
			0, 0, 0, 1;

		return M;
	}

private:
	double m_X_xt;
	double m_L_xt;
};

class F_yt : public Transformation
{
public:
	F_yt(double Y_yt, double L_yty, double L_ytz) :m_Y_yt(Y_yt), m_L_yty(L_yty), m_L_ytz(L_ytz){}
	~F_yt() {}
	matrix<double, 4, 4> getMatrix() const
	{
		matrix<double, 4, 4> M;

		M = 1, 0, 0, 0,
			0, 1, 0, m_L_yty + m_Y_yt,
			0, 0, 1, m_L_ytz,
			0, 0, 0, 1;

		return M;
	}

private:
	double m_Y_yt;
	double m_L_yty;
	double m_L_ytz;
};

class F_rr : public Transformation
{
public:
	F_rr(double phi_rr, double gamma, double phi, double l_IJ, double L_rrz) 
		:m_phi_rr(phi_rr), m_gamma(gamma), m_phi(phi), m_l_IJ(l_IJ), m_L_rrz(L_rrz){}
	~F_rr() {}
	matrix<double, 4, 4> getMatrix() const
	{
		matrix<double, 4, 4> M;
		
		double sin_plus = sin(m_gamma + m_phi);
		double sin_minus = sin(m_gamma - m_phi);

		M = cos(m_phi_rr), 0, sin(m_phi_rr), 0.25*(sin_plus-sin_minus)*m_l_IJ/sin(m_gamma),
			0, 1, 0, 0,
			-sin(m_phi_rr), 0, cos(m_phi_rr), 0.25*(sin_plus+sin_minus-2*sin(m_gamma))*m_l_IJ/cos(m_gamma)+m_L_rrz,
			0, 0, 0, 1;
		
		return M;
	}

private:
	double m_phi_rr;
	double m_gamma;
	double m_phi;
	double m_l_IJ;
	double m_L_rrz;
};

class F_t : public Transformation
{
public:
	F_t(double L_tz) :m_L_tz(L_tz) {}
	~F_t() {}
	matrix<double, 4, 4> getMatrix() const
	{
		matrix<double, 4, 4> M;

		M = 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, m_L_tz,
			0, 0, 0, 1;

		return M;
	}

private:
	double m_L_tz;
};

//matrix<double, 4, 4> finalMatrix(matrix<double,18,1> all_parameters)
//{
//	matrix<double, 4, 4> M = identity_matrix<double>(4);
//
//
//
//	return M;
//}
//
//
//int main() {
//	double theta_s = 0;
//	double theta_e = 0;
//	double psi_pr = 0;
//	double phi_rr = 0;
//	double Z_vt = 0;
//	double X_xt = 0;
//	double Y_yt = 0;
//
//	double phi = 0;
//
//	double L_e = -1000;
//	double L_vty = 209;
//	double L_vtz = -1079;
//	double L_xt = 40.5;
//	double L_yty = 791;
//	double L_ytz = 108;
//	double L_rrz = 7.74;
//	double L_tz = 82;
//	double gamma = pi / 6;
//	double l_IJ = 286;
//
//	Transformation* ptr;
//	matrix<double, 4, 4> F1, F2, F3, F4, F5, F6, F7;
//	matrix<double, 4, 1> Vector1, Vector2;
//
//	Vector1 = 0, 0, 82, 1;
//
//	ptr = new F_vt(L_vty, L_vtz);
//	F3 = ptr->getMatrix();
//	delete ptr;
//
//	for (double a = -0.1; a <= 0.1; a += 0.02)
//	{
//		double a_rad = a * pi / 180;
//		ptr = new F_s(theta_s + a_rad);
//		F1 = ptr->getMatrix();
//		delete ptr;
//
//		for (double b = -0.1; b <= 0.1; b += 0.02)
//		{
//			double b_rad = b * pi / 180;
//			ptr = new F_e(theta_e + b_rad, -1000);
//			F2 = ptr->getMatrix();
//			delete ptr;
//
//			for (double c = -0.01; c <= 0.01; c += 0.002)
//			{
//				double c_rad = c * pi / 180;
//				for (double d = -0.04; d <= 0.04; d += 0.008)
//				{
//					ptr = new F_pr(psi_pr + c_rad, Z_vt + d);
//					F4= ptr->getMatrix();
//					delete ptr;
//
//					for (double e = -0.04; e <= 0.04; e += 0.008)
//					{
//						ptr = new F_xt(X_xt + e, 40.5);
//						F5 = ptr->getMatrix();
//						delete ptr;
//
//						for (double f = -0.04; f <= 0.04; f += 0.008)
//						{
//							ptr = new F_yt(Y_yt + f, 791, 108);
//							F6 = ptr->getMatrix();
//							delete ptr;
//
//							for (double g = -0.01; g <= 0.01; g += 0.002)
//							{
//								double g_rad = g * pi / 180;
//								ptr = new F_rr(phi_rr + g, pi / 6, 0, 286, 7.74);
//								F7 = ptr->getMatrix();
//								delete ptr;
//
//								Vector2 = F1 * (F2 * (F3 * (F4 * (F5 * (F6 * (F7 * Vector1))))));
//
//								cout << a << " " << b << " " << c << " " << d << " " << e << " " << f << " " << g << " " 
//									<< Vector2(0)* Vector2(0)+ Vector2(1) * Vector2(1)+ Vector2(2) * Vector2(2) << endl;
//							}
//						}
//					}
//				}
//			}
//		}
//	}
//
//	/*
//	Transformation* ptr = new F_s(theta_s);
//	F *= ptr->getMatrix();
//	//cout << F << endl;
//	delete ptr;
//
//	ptr = new F_e(theta_e, L_e);
//	F *= ptr->getMatrix();
//	//cout << F << endl;
//	delete ptr;
//
//	ptr = new F_vt(L_vty, L_vtz);
//	F *= ptr->getMatrix();
//	//cout << F << endl;
//	delete ptr;
//
//	ptr = new F_pr(psi_pr, Z_vt);
//	F *= ptr->getMatrix();
//	//cout << F << endl;
//	delete ptr;
//
//	ptr = new F_xt(X_xt, L_xt);
//	F *= ptr->getMatrix();
//	//cout << F << endl;
//	delete ptr;
//
//	ptr = new F_yt(Y_yt, L_yty, L_ytz);
//	F *= ptr->getMatrix();
//	//cout << F << endl;
//	delete ptr;
//
//	ptr = new F_rr(phi_rr, gamma, phi, l_IJ, L_rrz);
//	F *= ptr->getMatrix();
//	//cout << F << endl;
//	delete ptr;
//
//	ptr = new F_t(L_tz);
//	F *= ptr->getMatrix();
//	//cout << F << endl;
//	delete ptr;
//	
//	double distant_squared = F(0, 3) * F(0, 3) + F(1, 3) * F(1, 3) + F(2, 3) * F(2, 3);
//	//cout << distant_squared << endl;
//	*/
//
//	return 0;
//}
