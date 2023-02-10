#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>

using std::cout;
using std::cin;
using std::endl;

class RobotModel {    
    private:
        Eigen::Vector3d robotPos;
        Eigen::Matrix3d wTr;
        Eigen::Vector3d pulleyLeft; 
        Eigen::Vector3d pulleyRight;
        Eigen::Vector3d efCornerPos_r;
        double wireDiam, q0, Lleft, Lright, Lleft0, Lright0, qRight0, qLeft0, coeff;
    public:
        RobotModel(Eigen::Vector3d robotPos, Eigen::Vector3d pulleyLeft,  Eigen::Vector3d pulleyRight, double wireDiam);
        RobotModel();
        /* Calculate motor positions Qs from desired position Xd */
        void inverseKinematics(Eigen::Vector3d Xd, int &qL, int &qR);
        /*  */
        double F(double &xIn, double deltaL);
        /*  */
        double GradF(double &xIn, double deltaL);
        /*  */
        void Q(double deltaL, int &qOut);
};