#include <eigen3/Eigen/Dense>

using Eigen::Vector3d;

class TrajectoryGenerator {
    public:
        TrajectoryGenerator();
        TrajectoryGenerator(Vector3d pis, Vector3d pfs, double Dt, double ti);
        Eigen::Matrix<double, 6, 1> getPol(double pi, double pf);
        double p(double t, Eigen::Matrix<double, 6, 1> a);
        double dp(double t, Eigen::Matrix<double, 6, 1> a);
        Vector3d X(double t);
        Vector3d dX(double t);
    private:
        Vector3d pis;
        Vector3d pfs;
        double ti;
        double Dt;
        Eigen::Matrix<double, 6, 1> polX;
        Eigen::Matrix<double, 6, 1> polY;
};