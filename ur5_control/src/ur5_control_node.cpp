#include <ros/ros.h>
#include <ur5_control/ur5_pos.h>
#include <ur5_control/ur5_dke_pos.h>
#include "std_msgs/Float64MultiArray.h"
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
ur5_control::ur5_pos ur5_pos_msg;
ur5_control::ur5_dke_pos ur5_dke_pos_msg;
std_msgs::Float64MultiArray current_pos;

std::vector<sensor_msgs::JointState> joint_states_data;
// 回调函数，当收到新的joint_states消息时调用
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // 将接收到的消息存储到数组中
    int i = 0;
    for (double joint : msg->position)
    {
        switch(i){
            case 0:
            {
                current_pos.data[2] = joint;
                break;
            }
            case 1:
            {
                current_pos.data[1] = joint;
                break;
            }
            case 2:
            {
                current_pos.data[0] = joint;
                break;
            }
            default:
            {
                current_pos.data[i] = joint;
                break;
            }
        }
        i++;
    }
}

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "ur5_control_node");
    ros::NodeHandle n;
    ros::Publisher ur5_pub = n.advertise<ur5_control::ur5_pos>("ur5_pos", 1000);
    ros::Publisher ur5_dke_pub = n.advertise<ur5_control::ur5_dke_pos>("ur5_dke_pos", 1000);
    ros::Subscriber joint_states_sub = n.subscribe("joint_states", 1000, jointStatesCallback);
    //pos initialization
    ur5_pos_msg.arm1 = 0.0f;
    ur5_pos_msg.arm2 = 0.0f;
    ur5_pos_msg.arm3 = 0.0f;
    ur5_pos_msg.arm4 = 0.0f;
    ur5_pos_msg.arm5 = 0.0f;
    ur5_pos_msg.arm6 = 0.0f;
    current_pos.data.push_back(0.0);
    current_pos.data.push_back(0.0);
    current_pos.data.push_back(0.0);
    current_pos.data.push_back(0.0);
    current_pos.data.push_back(0.0);
    current_pos.data.push_back(0.0);
    int command = -1;
    while(ros::ok())
    { 
        std::cout<<"+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<std::endl;
        std::cout<<"Enter your command!"<<std::endl;
        std::cout<<"Enter 0 to break!"<<std::endl;
        std::cout<<"Enter 1 to move ur5 in joint space!"<<std::endl;
        std::cout<<"-----------------------------------------------------------"<<std::endl;

        while (std::cin >> command)
        {
            std::cout<<"+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"<<std::endl;
            std::cout<<"Enter your command!"<<std::endl;
            std::cout<<"Enter 0 to break!"<<std::endl;
            std::cout<<"Enter 1 to move ur5 in joint space!"<<std::endl;
            std::cout<<"Enter 2 to move ur5 in dke space!"<<std::endl;
            std::cout<<"-----------------------------------------------------------"<<std::endl;
            
            if (command == 0)
            {
                std::cout<<"See you again!"<<std::endl;
                return 0;
            }
            else if(command == 1)
            {
                std::cout << "Enter joint values for arm1 to arm6: "<<std::endl;
                std::cin >> ur5_pos_msg.arm1 >> ur5_pos_msg.arm2 >> ur5_pos_msg.arm3 >> ur5_pos_msg.arm4 >> ur5_pos_msg.arm5 >> ur5_pos_msg.arm6;
                ur5_pub.publish(ur5_pos_msg);
            }
            else if(command == 2)
            {
                double x, y, z;
                std::cout << "请输入目标末端位置:x, y, z "<<std::endl;
                std::cin >> x >> y >> z;
                double qw, qx, qy, qz;
                std::cout << "请输入四元数 (w, x, y, z): ";
                std::cin  >> qx >> qy >> qz >> qw;
                ur5_dke_pos_msg.x = x;
                ur5_dke_pos_msg.y = y;
                ur5_dke_pos_msg.z = z;
                ur5_dke_pos_msg.qw = qw;
                ur5_dke_pos_msg.qx = qx;
                ur5_dke_pos_msg.qy = qy;
                ur5_dke_pos_msg.qz = qz;
                // 构造四元数对象
                // Eigen::Quaterniond quaternion(qw, qx, qy, qz);

                // // 规范化四元数（确保其为单位四元数）
                // quaternion.normalize();

                // // 将四元数转化为旋转矩阵
                // Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();

                // // 构造变换矩阵 (4x4)
                // Eigen::Matrix4d transformationMatrix = Eigen::Matrix4d::Identity();
                // transformationMatrix.block<3, 3>(0, 0) = rotationMatrix;
                // transformationMatrix(0, 3) = x;
                // transformationMatrix(1, 3) = y;
                // transformationMatrix(2, 3) = z;
                // ur5_dke_pos_msg.r11 = transformationMatrix(0, 0);
                // ur5_dke_pos_msg.r12 = transformationMatrix(0, 1);
                // ur5_dke_pos_msg.r13 = transformationMatrix(0, 2);
                // ur5_dke_pos_msg.r21 = transformationMatrix(1, 0);
                // ur5_dke_pos_msg.r22 = transformationMatrix(1, 1);
                // ur5_dke_pos_msg.r23 = transformationMatrix(1, 2);
                // ur5_dke_pos_msg.r31 = transformationMatrix(2, 0);
                // ur5_dke_pos_msg.r32 = transformationMatrix(2, 1);
                // ur5_dke_pos_msg.r33 = transformationMatrix(2, 2);
                // ur5_dke_pos_msg.t14 = transformationMatrix(0, 3);
                // ur5_dke_pos_msg.t24 = transformationMatrix(1, 3);
                // ur5_dke_pos_msg.t34 = transformationMatrix(2, 3);
                // ur5_dke_pos_msg.r41 = 0.0;
                // ur5_dke_pos_msg.r42 = 0.0;
                // ur5_dke_pos_msg.r43 = 0.0;
                // ur5_dke_pos_msg.t44 = 1.0;
                // ur5_dke_pos_msg.arm1 = current_pos.data[0];
                // ur5_dke_pos_msg.arm2 = current_pos.data[1];
                // ur5_dke_pos_msg.arm3 = current_pos.data[2];
                // ur5_dke_pos_msg.arm4 = current_pos.data[3];
                // ur5_dke_pos_msg.arm5 = current_pos.data[4];
                // ur5_dke_pos_msg.arm6 = current_pos.data[5];
                ur5_dke_pub.publish(ur5_dke_pos_msg);
            }
            else
            {
                std::cout<<"wrong command !!!";
            }
        }
        return 0;
    }

}
