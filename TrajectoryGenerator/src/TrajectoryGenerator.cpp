#include "TrajectoryGenerator.h"

TrajectoryGenerator::TrajectoryGenerator() {}

TrajectoryGenerator::TrajectoryGenerator(Vector2d pis, Vector2d pfs, double Dt, double ti) {
    this->Dt = Dt;
    this->ti = ti;
    this->polX = this->getPol(pis(0), pfs(0));
    this->polX = this->getPol(pis(1), pfs(1));
}

Eigen::Matrix<double, 6, 1> TrajectoryGenerator::getPol(double pi, double pf) {
    Eigen::Matrix<double, 6, 1> coef = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 1> L = Eigen::Matrix<double, 6, 1>::Zero();
    L << pi, pf, 0, 0, 0, 0;
    Eigen::Matrix<double,6,6> A = Eigen::Matrix<double,6,6>::Zero(); 
    A <<  0, 0, 0, 0, 0, 1,
        pow(this->Dt, 5), pow(this->Dt, 4), pow(this->Dt, 3), pow(this->Dt, 2), this->Dt, 1,
        0, 0, 0, 0, 1, 0,
        5*pow(this->Dt, 4), 4*pow(this->Dt, 3), 3*pow(this->Dt, 2), 2*(this->Dt), 1, 0,
        0, 0, 0, 2, 0, 0,
        20*pow(this->Dt, 3), 12*pow(this->Dt, 2), 6*(this->Dt), 2, 0, 0;
    coef = (A.inverse())*L;

    return coef;
}

double TrajectoryGenerator::p(double t, Eigen::Matrix<double, 6, 1> a) {
    const double td = t - this->ti;

    const double pos = a(0)*pow(td, 5) + a(1)*pow(td, 4) + a(2)*pow(td, 3) + a(3)*pow(td, 2) + a(4)*td + a(5);

    return pos;
}

double TrajectoryGenerator::dp(double t, Eigen::Matrix<double, 6, 1> a) {
    const double td = t - this->ti;

    const double pos = 5*a(0)*pow(td, 4) + 4*a(1)*pow(td, 3) + 3*a(2)*pow(td, 2) + 2*a(3)*pow(td, 1) + a(4);

    return pos;
}

Eigen::Vector2d TrajectoryGenerator::X(double t){
  return Eigen::Vector2d(this->p(t, this->polX), this->p(t, this->polY));
}

Eigen::Vector2d TrajectoryGenerator::dX(double t){
  return Eigen::Vector2d(this->dp(t, this->polX), this->dp(t, this->polY));
}


