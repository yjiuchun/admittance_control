#include <ros/ros.h>
#include "admittance_pos/Admittance_pos.h"





int main(int argc, char** argv)
{

    ros::init(argc, argv, "admittance_node");

    ros::NodeHandle nh;
    double frequency = 100.0;

    // Parameters
    std::string topic_arm_state;
    std::string topic_arm_command;
    std::string topic_wrench_state;
    std::string base_link;
    std::string end_link;

    std::vector<double> M;
    std::vector<double> D;
    std::vector<double> K;
    std::vector<double> init_desired_pose;
    
    double arm_max_vel;
    double arm_max_acc;
    if (!nh.getParam("mass_arm", M)) { ROS_ERROR("Couldn't retrieve the desired mass of the arm."); return -1; }
    if (!nh.getParam("damping_arm", D)) { ROS_ERROR("Couldn't retrieve the desired damping of the coupling."); return -1; }
    if (!nh.getParam("stiffness_coupling", K)) { ROS_ERROR("Couldn't retrieve the desired stiffness of the coupling."); return -1; }
    if (!nh.getParam("arm_max_vel", arm_max_vel)) { ROS_ERROR("Couldn't retrieve the max velocity for the arm."); return -1;}
    if (!nh.getParam("arm_max_acc", arm_max_acc)) { ROS_ERROR("Couldn't retrieve the max acceleration for the arm."); return -1;}
    if (!nh.getParam("base_link", base_link)) { ROS_ERROR("Couldn't retrieve the base_link."); return -1; }
    if (!nh.getParam("end_link", end_link)) { ROS_ERROR("Couldn't retrieve the end_link."); return -1; } 
    if (!nh.getParam("init_pose", init_desired_pose)) { ROS_ERROR("Couldn't retrieve the init_desired_pose."); return -1; } 
    if (!nh.getParam("arm_max_vel", arm_max_vel)) { ROS_ERROR("Couldn't retrieve the arm_max_vel."); return -1; } 
    if (!nh.getParam("arm_max_acc", arm_max_acc)) { ROS_ERROR("Couldn't retrieve the arm_max_acc."); return -1; } 

    AdmittancePos admittancepos(
        nh,
        100,
        "joint_states",
        "ur5_dke_pos",
        "ft_sensor_topic",
        M,D,K,
        base_link,
        end_link,
        init_desired_pose,
        arm_max_acc,
        arm_max_vel
    ); 
    admittancepos.run();
    while(1)
    {
        //TODO : Implement the admittance control algorithm
    }
    return 0;
}

