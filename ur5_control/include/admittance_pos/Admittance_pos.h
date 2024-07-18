/*
 * @Author: Yangjiuchun 
 * @Date: 2024-7-14 16:26:00 
 */

#ifndef ADMITTANCE_POS_H
#define ADMITTANCE_POS_H
#include "ros/ros.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "std_msgs/Float64.h"
#include <sensor_msgs/JointState.h>
#include "ur5_control/ur5_dke_pos.h"
#include "ur_kin.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/WrenchStamped.h"
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

class AdmittancePos
{
    protected:
    //Ros
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;
    //admittance param
    Matrix3d Md_;
    Matrix3d Dd_;
    Matrix3d Kd_;

    //Subscribers
    ros::Subscriber sub_joint_states_;
    ros::Subscriber sub_final_target_pose_;
    ros::Subscriber sub_current_end_wrench_;
    //Publishers
    ros::Publisher pub_joint1_command;
    ros::Publisher pub_joint2_command;
    ros::Publisher pub_joint3_command;
    ros::Publisher pub_joint4_command;
    ros::Publisher pub_joint5_command;
    ros::Publisher pub_joint6_command;

    //Variables
    std::vector<double> current_end_position_;              //当前末端位置
    Eigen::Quaterniond  current_end_orientation_;           //当前末端姿态
    std_msgs::Float64MultiArray current_joints_angle_;      //当前各关节角度
    ur5_control::ur5_dke_pos current_target_end_pose_;      //当前期望的末端位姿
    Eigen::Quaterniond  current_target_end_orientation_;    //当前期望的末端姿态
    Eigen::Vector3d current_target_end_position_;           //当前期望的末端位置
    Eigen::Vector3d current_target_end_velocity_;           //当前期望的末端速度
    Eigen::Vector3d current_target_end_acceleration_;       //当前期望的末端加速度
    Eigen::Vector3d current_base_wrench_;                   //当前base_link坐标系下受力
    ur5_control::ur5_dke_pos final_target_end_pose_;        //最终期望的末端位姿
    Eigen::Vector3d final_target_end_position_;             //最终期望的末端位置

    std::vector<std_msgs::Float64> desire_joints_commmand_; //解算后的发送给控制器的关节角度        

    std::string   base_link_;
    std::string   end_link_;
    //Listener
    tf::TransformListener listener_ft_;

    //
    double max_accl_;
    double max_vel_;

    public:
    AdmittancePos(
        ros::NodeHandle &n,
        double frequency,
        std::string current_joint_states_topic_name,
        std::string final_target_end_pose_topic_name,
        std::string current_end_wrench_topic_name,
        std::vector<double> M,
        std::vector<double> D,
        std::vector<double> K,
        std::string base_link,
        std::string end_link,
        std::vector<double> init_pose,
        double max_accl,
        double max_vel
        );
    ~AdmittancePos(){};


    //

    void send_to_controller(std::vector<std_msgs::Float64> commands)
    {
        pub_joint1_command.publish(commands[0]);
        pub_joint2_command.publish(commands[1]);
        pub_joint3_command.publish(commands[2]);
        pub_joint4_command.publish(commands[3]);
        pub_joint5_command.publish(commands[4]);
        pub_joint6_command.publish(commands[5]);
        // std::cout<<"sended"<<std::endl;
    }

    private:
    void current_joints_state_callback(const sensor_msgs::JointState::ConstPtr& msg);
    void final_target_end_pose_callback(const ur5_control::ur5_dke_pos::ConstPtr& msg);
    void current_end_wrench_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);

    //逆向运动学
    void inverse_solver(ur5_control::ur5_dke_pos current_end_command);
    //过圈函数
    double rad_format(double data);
    float squre_error(float data1, float data2);

    //选择最优逆解
    void solver_selector(double* q_sols, int num_sols);

    //导纳控制
    void admittance_controller();
    bool get_rotation_matrix(Matrix6d & rotation_matrix,
        tf::TransformListener & listener,
        std::string from_frame,
        std::string to_frame);
    void wait_for_transformations();
    public:
    void run();

};



#endif