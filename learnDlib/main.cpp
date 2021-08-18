//#include <iostream>
//#include <Eigen/Dense>
//#include "test3.h"
//
//using namespace std;
//using namespace Eigen;

//int main()
//{
//	VectorXd param = VectorXd::Zero(8);
//	param << 1, 2, 3, 4, 5, 6, 7, 8;
//
//	Vector3d XYZ = XYZ_max(param, -0.4, 0.4, -0.4, 0.4, -0.4, 0.4);
//	double f_max = f_XYZ_max(param, -0.4, 0.4, -0.4, 0.4, -0.4, 0.4);
//
//	cout << XYZ << endl;
//	cout << f_max << endl;
//	//2021.7.12 修改：需要判断极值点是极大值还是极小值
//	//2021.7.13 修改完成
//
//	return 0;
//}


//2021.7.13
//例子：C++读写csv文件
//#include <iostream>
//#include <string>
//#include <vector>
//#include <fstream>
//#include <sstream>
//
//using namespace std;
//
//
//int main()
//{
//	// 写文件
//	ofstream outFile;
//	outFile.open("data.csv", ios::out); // 打开模式可省略
//	outFile << "name" << ',' << "age" << ',' << "hobby" << endl;
//	outFile << "Mike" << ',' << 18 << ',' << "paiting" << endl;
//	outFile << "Tom" << ',' << 25 << ',' << "football" << endl;
//	outFile << "Jack" << ',' << 21 << ',' << "music" << endl;
//	outFile.close();
//
//	// 读文件
//	ifstream inFile("data.csv", ios::in);
//	string lineStr;
//	vector<vector<string>> strArray;
//	while (getline(inFile, lineStr))
//	{
//		// 打印整行字符串
//		cout << lineStr << endl;
//		// 存成二维表结构
//		stringstream ss(lineStr);
//		string str;
//		vector<string> lineArray;
//		// 按照逗号分隔
//		while (getline(ss, str, ','))
//			lineArray.push_back(str);
//		strArray.push_back(lineArray);
//	}
//
//	getchar();
//	return 0;
//}


#include <iostream>
#include <math.h>
#include <dlib/matrix.h>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "test1.h"
#include "test3.h"

using namespace dlib;
using namespace std;

int main()
{
	ofstream outFile;
	outFile.open("data.csv", ios::out);
	outFile << setiosflags(ios::fixed) << setprecision(4);

	double theta_s = 0;
	double theta_e = 0;
	double psi_pr = pi / 60;
	double phi_rr = pi / 60;
	double Z_vt = 650;
	double X_xt = 0;
	double Y_yt = 0;

	double theta_s_e = theta_s + theta_e;

	matrix<double, 4, 4> F1_2, F3_4_5_6, F7;
	matrix<double, 4, 1> Vector1, Vector2; //Vector2 = F1_2 * F3_4_5_6 * F7 * Vector1
	matrix<double, 3, 4> D;
	VectorXd param = VectorXd::Zero(8);
	Vector3d XYZ;
	double f_max;
	matrix<double, 3, 1> XYZ0;


	double temp = 0;

	Vector1 = 0, 0, 82, 1;
	XYZ0 = XYZ_0(theta_s, theta_e, psi_pr, phi_rr, Z_vt, X_xt, Y_yt, Vector1);
	//cout << XYZ0 << endl;

	for (double a = -0.1 /*-0.1*/; a < 0.1001/*0.1001*/; a += 0.01)
	{
		double a_rad = a * pi / 180;

		for (double b = -0.1 /*-0.1*/; b < 0.1001/*0.1001*/; b += 0.01)
		{
			double b_rad = b * pi / 180;
			double a_b_rad = a_rad + b_rad;

			F1_2 = cos(theta_s_e + a_b_rad), -sin(theta_s_e + a_b_rad), 0, 1000 * sin(theta_s + a_rad),
				sin(theta_s_e + a_b_rad), cos(theta_s_e + a_b_rad), 0, -1000 * cos(theta_s + a_rad),
				0, 0, 1, 0,
				0, 0, 0, 1;

			for (double g = -0.01; g < 0.01001; g += 0.001)
			{
				double g_rad = g * pi / 180;

				double element_03 = (sin(pi / 6 + phi_rr + g_rad) - sin(pi / 6 - phi_rr - g_rad)) * 286 / (4 * sin(pi / 6));
				double element_23 = 7.74 + (sin(pi / 6 + phi_rr + g_rad) + sin(pi / 6 - phi_rr - g_rad) - 2 * sin(pi / 6)) 
					* 286 / (4 * cos(pi / 6));

				F7 = cos(phi_rr + g_rad), 0, sin(phi_rr + g_rad), element_03,
					0, 1, 0, 0,
					-sin(phi_rr + g_rad), 0, cos(phi_rr + g_rad), element_23,
					0, 0, 0, 1;

				for (double c = -0.01; c < 0.01001; c += 0.001)
				{
					double c_rad = c * pi / 180;

					D = getD(F1_2, F7, psi_pr, c_rad, Vector1);
					param << D(0, 0), D(0, 1), D(0, 3) - XYZ0(0), D(1, 0), D(1, 1), D(1, 3) - XYZ0(1), D(2, 1), D(2, 3) - XYZ0(2);

					XYZ = XYZ_max(param, X_xt - 0.04, X_xt + 0.04, Y_yt - 0.04, Y_yt + 0.04, Z_vt - 0.04, Z_vt + 0.04);
					f_max = f_XYZ_max(param, X_xt - 0.04, X_xt + 0.04, Y_yt - 0.04, Y_yt + 0.04, Z_vt - 0.04, Z_vt + 0.04);

					double X_atMax, Y_atMax, Z_atMax;
					X_atMax = D(0, 0) * XYZ[0] + D(0, 1) * XYZ[1] + D(0, 3);
					Y_atMax = D(1, 0) * XYZ[0] + D(1, 1) * XYZ[1] + D(1, 3);
					Z_atMax = D(2, 1) * XYZ[1] + XYZ[2] + D(2, 3);

					outFile << a << ',' << b << ',' << c << ',' << XYZ[2] << ',' << XYZ[0] << ',' << XYZ[1] << ',' << g << ','
						<< sqrt(f_max) << ',' << "    " << ','
						<< X_atMax << ',' << Y_atMax << ',' << Z_atMax << ',' << "    " << ','
						<< XYZ0(0) << ',' << XYZ0(1) << ',' << XYZ0(2) << endl;

				}
			}
		}
	}

	outFile.close();

	return 0;
}