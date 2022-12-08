#include <eigen3/Eigen/Dense>

using Eigen::Vector2d;

class TrajectoryGenerator {
    public:
        TrajectoryGenerator();
        TrajectoryGenerator(Vector2d pis, Vector2d pfs, double Dt, double ti);
        Eigen::Matrix<double, 6, 1> getPol(double pi, double pf);
        double p(double t, Eigen::Matrix<double, 6, 1> a);
        double dp(double t, Eigen::Matrix<double, 6, 1> a);
        Vector2d X(double t);
        Vector2d dX(double t);
    private:
        Vector2d pis;
        Vector2d pfs;
        double ti;
        double Dt;
        Eigen::Matrix<double, 6, 1> polX;
        Eigen::Matrix<double, 6, 1> polY;
};