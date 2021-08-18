#pragma once

double f(double delta, double A, double B, double Rx, double Ry, double lLR);
double df(double delta, double A, double B, double Rx, double Ry, double lLR);

double solveDeltaIter(double A, double B, double Rx, double Ry, double lLR);