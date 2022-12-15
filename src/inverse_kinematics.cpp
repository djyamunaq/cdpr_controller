#include <iostream>
#include <cmath>

using namespace std;

double Inverse_Kinematic_Model(int q) {
    long double coeff = 0.5323;
    long double pi = 3.1415;
    long double a = 0.2*coeff;
    long double k = a/2/pi;
    long double q0_rad = 0.6/k;
    long double q1_rad = q0_rad + (2*pi*q / 7400.0);

    cout << q0_rad << endl;
    cout << q1_rad << endl;

    long double L = a/2/pi * (q1_rad/2 * sqrt( pow(q1_rad,2) + 1) + 0.5 * log(q1_rad +  sqrt(pow(q1_rad,2)+1)) - q0_rad/2 *  sqrt(pow(q0_rad,2)+1) - 0.5 * log(q0_rad +  sqrt( pow(q0_rad, 2)+1)));
    return L;
}

int main() {
    cout << "Hello world!" << endl;
    int q = 2*7400;
    double L = Inverse_Kinematic_Model(q);
    cout << "L = " << L << endl;
    cout << pow(4,2) <<endl;

    return 0;
}