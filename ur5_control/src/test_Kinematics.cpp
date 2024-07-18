#include"UR5_Inverse_Kinematics_Solver.hpp"
#include <stdio.h>
#include <iomanip>
#include <ros/ros.h>

double* dada()
{

    double *a = new double[6];
    
    //  = {1,2,3,4,5,6};
    return a;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_Kinematics");
    ros::NodeHandle nh;
    Ur_Kinematics kinematics;

    double theta[6] = {0.7,-0.32,-0.87,0.1,0.1,0.1};


    Eigen::MatrixXd matrix44;

    matrix44 = kinematics.forward(theta);

    std::cout << matrix44 << std::endl;

    Ur_Kinematics::inverse_8theta inverse_theta;

    inverse_theta = kinematics.inverse(matrix44);


   
    for (int i = 0; i < 8; i++) {
        std::cout << "第" << std::setw(2) << i << "组解: ";

        for (int j = 0; j < 6; j++) {
            std::cout << std::setw(15) << std::right << inverse_theta.th[j][i] << ' ';
        }

        std::cout << std::endl;
    }

    return 0;

}