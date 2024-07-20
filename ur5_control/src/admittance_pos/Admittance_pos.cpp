/*
 * @Author: Yangjiuchun 
 * @Date: 2024-7-14 16:26:00 
 */
#include "admittance_pos/Admittance_pos.h"
#include <chrono>

AdmittancePos::AdmittancePos(        
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
        ) :
        nh_(n), loop_rate_(frequency), current_end_position_(3), desire_joints_commmand_(6),
        Md_(M.data()), Dd_(D.data()), Kd_(K.data()),base_link_(base_link),end_link_(end_link),
        max_accl_(max_accl), max_vel_(max_vel)
        {
            //Subscribers
            sub_joint_states_ =             nh_.subscribe(current_joint_states_topic_name, 5, 
                &AdmittancePos::current_joints_state_callback, this,ros::TransportHints().reliable().tcpNoDelay());
            sub_final_target_pose_ =        nh_.subscribe(final_target_end_pose_topic_name, 5, 
                &AdmittancePos::final_target_end_pose_callback, this,ros::TransportHints().reliable().tcpNoDelay());
            sub_current_end_wrench_ =       nh_.subscribe(current_end_wrench_topic_name, 5, 
                &AdmittancePos::current_end_wrench_callback, this,ros::TransportHints().reliable().tcpNoDelay());
            //Publisher
            pub_joint1_command =            nh_.advertise<std_msgs::Float64>("/joint1_pos_controller/command",1000);
            pub_joint2_command =            nh_.advertise<std_msgs::Float64>("/joint2_pos_controller/command",1000);
            pub_joint3_command =            nh_.advertise<std_msgs::Float64>("/joint3_pos_controller/command",1000);
            pub_joint4_command =            nh_.advertise<std_msgs::Float64>("/joint4_pos_controller/command",1000);
            pub_joint5_command =            nh_.advertise<std_msgs::Float64>("/joint5_pos_controller/command",1000);
            pub_joint6_command =            nh_.advertise<std_msgs::Float64>("/joint6_pos_controller/command",1000);

            //current joint angle init
            current_joints_angle_.data.resize(6);
            for(int i=0;i<6;i++)
            {
                current_joints_angle_.data[i] = 0.0;
                desire_joints_commmand_[i].data = 0.0;
            }

            //init current target var
            current_target_end_position_ << 0.0, 0.0, 0.0;
            current_target_end_velocity_ << 0.0, 0.0, 0.0;
            current_target_end_acceleration_ << 0.0, 0.0, 0.0;
            final_target_end_position_ << 0.0, 0.0, 0.0;

            //init target pose
            final_target_end_pose_.x = init_pose[0];
            final_target_end_pose_.y = init_pose[1];
            final_target_end_pose_.z = init_pose[2];
            final_target_end_pose_.qw = init_pose[3];
            final_target_end_pose_.qx = init_pose[4];
            final_target_end_pose_.qy = init_pose[5];
            final_target_end_pose_.qz = init_pose[6];
            std::cout<<final_target_end_pose_<<std::endl;
            final_target_end_position_<<init_pose[0], init_pose[1], init_pose[2];
            wait_for_transformations();
        }

//!-                   INITIALIZATION                    -!//

void AdmittancePos::wait_for_transformations() {
  tf::TransformListener listener;
  Matrix6d rot_matrix;
  // Makes sure all TFs exists before enabling all transformations in the callbacks
  // while (!get_rotation_matrix(rot_matrix, listener, "world", base_link_)) {sleep(1);}
//   base_world_ready_ = true;
  // while (!get_rotation_matrix(rot_matrix, listener, base_link_, "world")) {sleep(1);}
//   world_arm_ready_ = true;
  while (!get_rotation_matrix(rot_matrix, listener, base_link_, end_link_)) {sleep(1);}
//   ft_arm_ready_ = true;
  ROS_INFO("The Force/Torque sensor is ready to use.");
}



                //-   -   -   -   -CallBack Fun-    -   -   -   -//
                
void AdmittancePos::current_joints_state_callback(const sensor_msgs::JointState::ConstPtr &msg)
{    
    // 将接收到的消息存储到数组中
    int i = 0;
    for (double joint : msg->position)
    {
        switch(i){
            case 0:
            {
                current_joints_angle_.data[2] = joint;
                break;
            }
            case 1:
            {
                current_joints_angle_.data[1] = joint;
                break;
            }
            case 2:
            {
                current_joints_angle_.data[0] = joint;
                break;
            }
            default:
            {
                current_joints_angle_.data[i] = joint;
                break;
            }
        }
        i++;
    }
}

void AdmittancePos::final_target_end_pose_callback(const ur5_control::ur5_dke_pos::ConstPtr& msg)
{
    final_target_end_pose_ = *msg;
    final_target_end_position_ << final_target_end_pose_.x, final_target_end_pose_.y, final_target_end_pose_.z;
    ROS_INFO("Get target command");
}

void AdmittancePos::current_end_wrench_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
    Vector6d wrench_ft_frame;
    Vector6d wrench_base_frame;
    Matrix6d rotation_ft_base;
    wrench_ft_frame <<  msg->wrench.force.x,msg->wrench.force.y,msg->wrench.force.z,0,0,0;
    get_rotation_matrix(rotation_ft_base, listener_ft_, base_link_, end_link_);
    wrench_base_frame = rotation_ft_base * wrench_ft_frame;
    current_base_wrench_ = wrench_base_frame.head<3>();
    current_base_wrench_[0] = -current_base_wrench_[0];
    current_base_wrench_[1] = -current_base_wrench_[1];
    current_base_wrench_[2] = current_base_wrench_[2];
    // for(int i=0;i<3;i++)
    // {
    //     if(abs(current_base_wrench_[i])<0.5)
    //     {
    //         current_base_wrench_[i] = 0;
    //     }
    //     current_base_wrench_[i] = 0;
    // }

    std::cout<<"force:"<<current_base_wrench_<<std::endl; 
    std::cout<<"=========================="<<std::endl;
}


//

void AdmittancePos::run()
{
    // TODO: Implement the admittance control algorithm
    while (nh_.ok()) {

        current_target_end_pose_ = final_target_end_pose_;
        admittance_controller();
        std::cout<<current_target_end_pose_<<std::endl;
        std::cout<<"0000000000000000000000000000"<<std::endl;
        inverse_solver(current_target_end_pose_);
        // for(int i=0;i<6;i++)
        // {
        //     std::cout<<desire_joints_commmand_[i].data<<" ";
        // }
        // std::cout<<std::endl;
        send_to_controller(desire_joints_commmand_);
        //机械臂逆向运动学求解
        ros::spinOnce();
        loop_rate_.sleep();
    }
}


//逆向运动学求解

void AdmittancePos::inverse_solver(ur5_control::ur5_dke_pos current_end_command)
{
    double q_sols[8*6];
    int num_sols;
    double* T = new double[16];
    // 构造四元数对象
    current_target_end_orientation_ = Eigen::Quaterniond(current_end_command.qw, 
                                                         current_end_command.qx, 
                                                         current_end_command.qy, 
                                                         current_end_command.qz);

    // 规范化四元数（确保其为单位四元数）
    current_target_end_orientation_.normalize();

    // 将四元数转化为旋转矩阵
    Eigen::Matrix3d rotationMatrix = current_target_end_orientation_.toRotationMatrix();

    // 构造变换矩阵 (4x4)
    Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
    transformationMatrix.block<3, 3>(0, 0) = rotationMatrix;
    transformationMatrix(0, 3) = current_end_command.x;
    transformationMatrix(1, 3) = current_end_command.y;
    transformationMatrix(2, 3) = current_end_command.z;
    T[0] = transformationMatrix(0, 0);
    T[1] = transformationMatrix(0, 1);
    T[2] = transformationMatrix(0, 2);
    T[3] = transformationMatrix(0, 3);
    T[4] = transformationMatrix(1, 0);
    T[5] = transformationMatrix(1, 1);
    T[6] = transformationMatrix(1, 2);
    T[7] = transformationMatrix(1, 3);
    T[8] = transformationMatrix(2, 0);
    T[9] = transformationMatrix(2, 1);
    T[10] = transformationMatrix(2, 2);
    T[11] = transformationMatrix(2, 3);
    T[12] = 0.0;
    T[13] = 0.0;
    T[14] = 0.0;
    T[15] = 1.0;

    auto start = std::chrono::high_resolution_clock::now();
    num_sols = ur_kinematics::inverse(T, q_sols);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    // ROS_INFO("Elapsed time: %f seconds", duration.count());
    // for(int i=0;i<num_sols;i++) 
    //                 printf("%1.6f %1.6f %1.6f %1.6f %1.6f %1.6f\n", 
    //                 rad_format(q_sols[i*6+0]), rad_format(q_sols[i*6+1]), 
    //                 rad_format(q_sols[i*6+2]), rad_format(q_sols[i*6+3]), 
    //                 rad_format(q_sols[i*6+4]), rad_format(q_sols[i*6+5]));
    solver_selector(q_sols, num_sols);
}

double AdmittancePos::rad_format(double data)
{
    while(data > 3.1415926)
        data -= 2*3.1415926;
    while(data < -3.1415926)
        data += 2*3.1415926;
    return data; 
}

float AdmittancePos::squre_error(float data1, float data2)
{
    return rad_format(data1-data2)*rad_format(data1-data2);
}

void AdmittancePos::solver_selector(double* q_sols, int num_sols)
{
    float min_squre_error = 1000.0;
    for(int j=0;j<num_sols;j++)
    {
        float current_error = 0.0;
        for(int k=0;k<6;k++)
        {
            current_error += squre_error(rad_format(q_sols[j*6+k]), current_joints_angle_.data[k]);
        }
        if(current_error < min_squre_error)
        {
            for(int q=0;q<6;q++)
            {
                if(q_sols[j*6+q] == 'nan')
                {
                    desire_joints_commmand_[q].data = 0.0;
                    
                }
                else
                {
                    desire_joints_commmand_[q].data = rad_format(q_sols[j*6+q]);
                }
            }
            min_squre_error = current_error;
        }
    }
}


//Fext = Md*(ddxd-ddx0) + Dd*(dxd-dx0) + Kd*(xd-x0)
void AdmittancePos::admittance_controller()
{
    current_target_end_acceleration_ = Md_.inverse() * (current_base_wrench_ - Dd_ * current_target_end_velocity_
                                        - Kd_ * (current_target_end_position_ - final_target_end_position_));
    for(int i=0;i<3;i++)
    {
        if(abs(current_target_end_acceleration_[i]) > max_accl_)
        {
            if(current_target_end_acceleration_[i] > 0)
                current_target_end_acceleration_[i] = max_accl_;
            else
                current_target_end_acceleration_[i] = -max_accl_;
            
            ROS_WARN("to big accl!!");
        }
    }
    // std::cout<<current_target_end_acceleration_<<std::endl;
    // std::cout<<"+++++++++++++++++++++++++++++++"<<std::endl;
    ros::Duration duration = loop_rate_.expectedCycleTime();
    current_target_end_velocity_ += current_target_end_acceleration_ * duration.toSec();
    for(int i=0;i<3;i++)
    {
        if(abs(current_target_end_velocity_[i]) > max_vel_)
        {
            if(current_target_end_velocity_[i] > 0)
                current_target_end_velocity_[i] = max_vel_;
            else
                current_target_end_velocity_[i] = -max_vel_;
            
            ROS_WARN("to big vel!!");
        }
    }
    // std::cout<<current_target_end_velocity_<<std::endl;
    // std::cout<<"-------------------------------"<<std::endl;
    current_target_end_position_ += current_target_end_velocity_ * duration.toSec();
    current_target_end_pose_.x = current_target_end_position_[0];
    current_target_end_pose_.y = current_target_end_position_[1];
    current_target_end_pose_.z = current_target_end_position_[2];
    // std::cout<<current_target_end_position_<<std::endl;
    // std::cout<<"============================"<<std::endl;


}

bool AdmittancePos::get_rotation_matrix(Matrix6d & rotation_matrix,
    tf::TransformListener & listener,
    std::string from_frame,
    std::string to_frame) {
  tf::StampedTransform transform;
  Matrix3d rotation_from_to;
  try {
    listener.lookupTransform(from_frame, to_frame,
                            ros::Time(0), transform);
    tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
    rotation_matrix.setZero();
    rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
    rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
  }
  catch (tf::TransformException ex) {
    rotation_matrix.setZero();
    ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << from_frame << " to: " << to_frame );
    return false;
  }
  return true;
}


