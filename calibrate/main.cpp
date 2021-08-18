#include <iostream>
#include <Eigen\Dense>
#include <string>
#include <fstream>
#include <sstream>
#include "motion.h"
#include "pitch1.h"

#define PI 3.1415926

using namespace std;
using namespace Eigen;

int main()
{
	//ofstream outFile;
	//outFile.open("Project1.csv", ios::out);
	int len = 10;
	double lON = 360;
	VectorXd angles(100);
	VectorXd lengths(100);
	VectorXd start(len);
	VectorXd actual(len);
	start << 250, 250, 432.7, 800, 655.8789, 800.3, 205.0325, 25, 119, 165 * PI / 180;
	actual << 250, 250, 432.7, 801, 655.8789, 800.3, 205.0325, 25, 119, 165 * PI / 180;

	for (int i = 0; i < 100; i++)
	{
		angles(i) = i * PI / 6000;
		lengths(i) = pitchAngleToLength(actual, angles(i), lON);
	}

	PitchMotion p(lON);
	int info = p.calibrate(angles, lengths, 100, start);
	cout << "actual: \n" << actual << endl;
	cout << "calibrated: \n" << start << endl;

	///*outFile.close();*/
	return 0;
}
