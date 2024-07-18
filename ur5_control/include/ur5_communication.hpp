#pragma once
/*  This file name is RosCommunication.h  */
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

/* This is a dedicated ROS communication class. There are three methods in this class  */
class RosCommunication
{
public:
    RosCommunication(std::string JointCommandTopic, std::string JointStateTopic, std::string FtSensorTopic, ros::NodeHandle &nh);
    RosCommunication() = delete;
    RosCommunication(const RosCommunication &) = delete;
    RosCommunication &operator=(const RosCommunication &) = delete;

public:
    //Send joint control commands(JointArray) to the JointCommandTopic
    // void SetJointPosition(const std_msgs::Float64MultiArray &JointArray);

    //Get a joint state message from the JointStateTopic
    sensor_msgs::JointState::ConstPtr GetJointState();

    //Get a force message from the FtSensorTopic
    geometry_msgs::WrenchStamped::ConstPtr GetForce();

    //tarj add mid point
    void AddTrajPoint(double time, const std_msgs::Float64MultiArray &start_pos ,const std_msgs::Float64MultiArray &target_pos, int point_num);

private:
    std::string JointCommandTopic;
    std::string JointStateTopic;
    std::string FtSensorTopic;
    ros::Publisher pub;
    ros::NodeHandle nh;
    trajectory_msgs::JointTrajectory traj_;
    trajectory_msgs::JointTrajectoryPoint point_;

};

RosCommunication::RosCommunication(std::string JointCommandTopic, std::string JointStateTopic, std::string FtSensorTopic, ros::NodeHandle &nh)
{
    this->JointCommandTopic = JointCommandTopic;
    this->JointStateTopic = JointStateTopic;
    this->FtSensorTopic = FtSensorTopic;
    this->nh = nh;
    this->pub = this->nh.advertise<trajectory_msgs::JointTrajectory>(this->JointCommandTopic, 10);
    //param init
    point_.positions.resize(6);
    point_.velocities.resize(6);
    point_.accelerations.resize(6);
    point_.effort.resize(6);
    traj_.joint_names.push_back("shoulder_pan_joint");
    traj_.joint_names.push_back("shoulder_lift_joint");
    traj_.joint_names.push_back("elbow_joint");
    traj_.joint_names.push_back("wrist_1_joint");
    traj_.joint_names.push_back("wrist_2_joint");
    traj_.joint_names.push_back("wrist_3_joint");
}

// void RosCommunication::SetJointPosition(const std_msgs::Float64MultiArray &JointArray)
// {
//     // while (this->pub.getNumSubscribers() < 1);
//     // for(int i=0;i<6;i++)
//     // {
//     //     point.positions[i] = JointArray.data[i];
//     // }   
//     // // 设置时间
//     // point.time_from_start = ros::Duration(2);
//     traj_.points.clear(); // 清空之前的轨迹点s
//     AddTrajPoint();
//     // // 将点添加到轨迹中
//     // traj.points.push_back(point);
//     // // 设置当前时间戳
//     // traj.header.stamp = ros::Time::now();
//     // // 发布轨迹消息
//     // this->pub.publish(traj);
//     ROS_INFO("done");
// }

sensor_msgs::JointState::ConstPtr RosCommunication::GetJointState()
{
    return ros::topic::waitForMessage<sensor_msgs::JointState>(this->JointStateTopic);
}

geometry_msgs::WrenchStamped::ConstPtr RosCommunication::GetForce()
{
    return ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(this->FtSensorTopic);
}

void RosCommunication::AddTrajPoint(double time, const std_msgs::Float64MultiArray &start_pos ,const std_msgs::Float64MultiArray &target_pos, int point_num)
{
    traj_.points.clear();
    for (int i = 0; i <= point_num; ++i)
    {
        double t = i * time / point_num;
        double ratio = static_cast<double>(i) / point_num;
        // trajectory_msgs::JointTrajectoryPoint point;
        for (int j = 0; j < 6; j++)
        {
            double a0 = start_pos.data[j];
            double a1 = 0; // 初速度为0
            double a2 = 3 * (target_pos.data[j] - start_pos.data[j]) / (time * time);
            double a3 = -2 * (target_pos.data[j] - start_pos.data[j]) / (time * time * time);
            double pos = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
            if(j == 2){
            std::cout<<pos<<std::endl;
            }
            point_.positions[j] = pos;
        }
        point_.time_from_start = ros::Duration(t+0.1);
        // std::cout<<t<<std::endl;

        traj_.points.push_back(point_);
        // 设置当前时间戳
        traj_.header.stamp = ros::Time::now();
        // 发布轨迹消息
        this->pub.publish(traj_);
        // ROS_INFO("ing");
        ros::Duration(0.001).sleep(); // 让控制器有时间处理命令
    }
    ROS_INFO("done");
    // for(int i=0;i<6;i++)
    // {
    //     point_.positions[i] = target_pos.data[i];
    // }   
    // // 设置时间
    // point_.time_from_start = ros::Duration(2);
    // traj_.points.clear(); // 清空之前的轨迹点s
    // // 将点添加到轨迹中
    // traj_.points.push_back(point_);
    // // 设置当前时间戳
    // traj_.header.stamp = ros::Time::now();
    // // 发布轨迹消息
    // this->pub.publish(traj_);

}