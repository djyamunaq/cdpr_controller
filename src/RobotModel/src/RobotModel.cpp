#include "RobotModel.h"
#include <math.h>

#define pi M_PI

RobotModel::RobotModel(Eigen::Vector3d robotPos, Eigen::Matrix3d wTr, Eigen::Vector3d pulleyLeft,  Eigen::Vector3d pulleyRight, Eigen::Vector3d efCornerPos_r, double wireDiam) {
    this->robotPos = robotPos;
    this->wTr = wTr;
    this->pulleyLeft = pulleyLeft;
    this->pulleyRight = pulleyRight;
    this->efCornerPos_r = efCornerPos_r;
    this->wireDiam = wireDiam;
    this->k = wireDiam/2/pi;
    this->tol = 1e-2;
    this->Niter = 100;
}        
        
RobotModel::RobotModel() {}

void RobotModel::inverseKinematics(Eigen::Vector3d Xd, std::vector<int> &Qs) {
    /* Calculate end-effector corner in world frame */
    Eigen::Vector3d efCornerPos_w = this->wTr * this->efCornerPos_r;
    /* Calculate wire lengths*/
    this->Lleft = (efCornerPos_w - this->pulleyLeft).norm(); 
    this->Lright = (efCornerPos_w - this->pulleyRight).norm(); 

    // int Qleft = Q(this->Lleft);
    int Qleft = 0;
    // int Qright = Q(this->Lright);
    int Qright = 0;

    Qs[0] = Qleft;
    Qs[1] = Qright;
}

double RobotModel::F(double &xIn, double &targetIn){
    double a = this->wireDiam;
    double phi_0_c = 0.6/this->k;

    return pow(a*(-phi_0_c*sqrt(pow(phi_0_c,2) + 1)/2 + (phi_0_c/2 + xIn/2)*sqrt(pow(phi_0_c + xIn,2) + 1) - 0.5*log(phi_0_c + sqrt(pow(phi_0_c,2) + 1)) + 0.5*log(phi_0_c + xIn + sqrt(pow(phi_0_c + xIn,2) + 1)))/(2*pi) - targetIn,2);
}

double RobotModel::GradF(double &xIn, double &targetIn){
    double a = this->wireDiam;
    double phi_0_c = 0.6/this->k;

    return a*(a*(-phi_0_c*sqrt(pow(phi_0_c,2) + 1)/2 + (phi_0_c/2 + xIn/2)*sqrt(pow(phi_0_c + xIn,2) + 1) - 0.5*log(phi_0_c + sqrt(pow(phi_0_c,2) + 1)) + 0.5*log(phi_0_c + xIn + sqrt(pow(phi_0_c + xIn,2) + 1)))/(2*pi) - targetIn)*((phi_0_c/2 + xIn/2)*(phi_0_c + xIn)/sqrt(pow(phi_0_c + xIn,2) + 1) + 0.5*((phi_0_c + xIn)/sqrt(pow(phi_0_c + xIn,2) + 1) + 1)/(phi_0_c + xIn + sqrt(pow(phi_0_c + xIn,2) + 1)) + sqrt(pow(phi_0_c + xIn,2) + 1)/2)/pi;
}

void RobotModel::Q(double &targetIn, int &qOut){
    double d0;
    double X1=0;
    double tol = this->tol;
    int Niter = this->Niter;
    for ( int i = 0; i < Niter; i++) {
        d0 = -this->GradF(X1, targetIn);
        X1 = X1 + k*d0;
        if (F(X1, targetIn) < tol) {break;}
    }   
    qOut = int(7400 * (X1/2/pi));
}