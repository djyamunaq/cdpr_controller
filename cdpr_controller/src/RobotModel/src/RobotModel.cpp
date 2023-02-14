#include "RobotModel.h"
#include <math.h>

#define pi M_PI

RobotModel::RobotModel(Eigen::Vector3d robotPos, Eigen::Vector3d pulleyLeft,  Eigen::Vector3d pulleyRight, double wireDiam) {
    this->wTr = wTr;
    this->pulleyLeft = pulleyLeft;
    this->pulleyRight = pulleyRight;
    this->wireDiam = wireDiam;
    this->Lleft0 = (robotPos - this->pulleyLeft).norm();
    this->Lleft = this->Lleft0;
    this->Lright0 = (robotPos - this->pulleyRight).norm();
    this->Lright = this->Lright0;
    this->qRight0 = 0;
    this->qLeft0 = 0;
    this->coeff = 0.5323;
}        
        
RobotModel::RobotModel() {}

void RobotModel::inverseKinematics(Eigen::Vector3d Xd, int &qL, int &qR) {
    /* Calculate wire deltaLs*/
    this->Lleft = (Xd - this->pulleyLeft).norm(); 
    this->Lright = (Xd - this->pulleyRight).norm(); 

    double DeltaLLeft = this->Lleft0 - this->Lleft;
    double DeltaLRight = this->Lright0 - this->Lright;

    int Qleft = 0;
    this->Q(DeltaLLeft, Qleft);
    int Qright = 0; 
    this->Q(DeltaLRight, Qright);

    qL = Qleft;
    qR = Qright;

    cout << this->Lleft << ' ' << this->Lright << endl;
}

double RobotModel::F(double &xIn, double deltaL){
    double a = this->wireDiam*this->coeff;
    double k = a/2/pi;
    double phi_0_c = 0.6/k;
    return pow(a*(-phi_0_c*sqrt(pow(phi_0_c,2) + 1)/2 + (phi_0_c/2 + xIn/2)*sqrt(pow(phi_0_c + xIn,2) + 1) - 0.5*log(phi_0_c + sqrt(pow(phi_0_c,2) + 1)) + 0.5*log(phi_0_c + xIn + sqrt(pow(phi_0_c + xIn,2) + 1)))/(2*pi) - deltaL,2);
}

double RobotModel::GradF(double &xIn, double deltaL){
    double a = this->wireDiam*this->coeff;
    double k = a/2/pi;
    double phi_0_c = 0.6/k;

    return a*(a*(-phi_0_c*sqrt(pow(phi_0_c,2) + 1)/2 + (phi_0_c/2 + xIn/2)*sqrt(pow(phi_0_c + xIn,2) + 1) - 0.5*log(phi_0_c + sqrt(pow(phi_0_c,2) + 1)) + 0.5*log(phi_0_c + xIn + sqrt(pow(phi_0_c + xIn,2) + 1)))/(2*pi) - deltaL)*((phi_0_c/2 + xIn/2)*(phi_0_c + xIn)/sqrt(pow(phi_0_c + xIn,2) + 1) + 0.5*((phi_0_c + xIn)/sqrt(pow(phi_0_c + xIn,2) + 1) + 1)/(phi_0_c + xIn + sqrt(pow(phi_0_c + xIn,2) + 1)) + sqrt(pow(phi_0_c + xIn,2) + 1)/2)/pi;
}

void RobotModel::Q(double deltaL, int &qOut){
    double d0;
    double X1=0.0;
    double tol = 1e-3;
    int Niter = 100;
    double tk = 10e-2;

    for ( int i = 0; i < Niter; i++) {
        d0 = -GradF(X1, deltaL);
        X1 = X1 + tk*d0;
        if (F(X1, deltaL) < tol) {break;}
    }   
    qOut = int(7400 * (X1/2/pi));
}