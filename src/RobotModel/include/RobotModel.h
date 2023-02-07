#include <eigen3/Eigen/Dense>
#include <vector>

class RobotModel {    
    private:
        Eigen::Vector3d robotPos;
        Eigen::Matrix3d wTr;
        Eigen::Vector3d pulleyLeft; 
        Eigen::Vector3d pulleyRight;
        Eigen::Vector3d efCornerPos_r;
        double wireDiam, q0, k, Lleft, Lright, tol;
        int Niter;
    public:
        RobotModel(Eigen::Vector3d RobotPos, Eigen::Matrix3d wTr, Eigen::Vector3d pulleyLeft,  Eigen::Vector3d pulleyRight, Eigen::Vector3d efCornerPos, double wireDiam);
        RobotModel();
        /* Calculate motor positions Qs from desired position Xd */
        void inverseKinematics(Eigen::Vector3d Xd, std::vector<int> &Qs);
        /*  */
        double F(double &xIn, double &targetIn);
        /*  */
        double GradF(double &xIn, double &targetIn);
        /*  */
        void Q(double &targetIn, int &qOut);
};