#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "ur5_arm_controller");
    ros::NodeHandle nh;

    // 创建一个发布器，发布到arm_group_controller的命令话题
    ros::Publisher arm_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_group_controller/command", 10);

    // 设置发布频率
    ros::Rate loop_rate(10);

    // 创建JointTrajectory消息
    trajectory_msgs::JointTrajectory traj;  
    traj.joint_names.push_back("shoulder_pan_joint");
    traj.joint_names.push_back("shoulder_lift_joint");
    traj.joint_names.push_back("elbow_joint");
    traj.joint_names.push_back("wrist_1_joint");
    traj.joint_names.push_back("wrist_2_joint");
    traj.joint_names.push_back("wrist_3_joint");

    // 创建一个JointTrajectoryPoint
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.resize(6);
    point.velocities.resize(6);
    point.accelerations.resize(6);
    point.effort.resize(6);

    // 设置关节位置（示例值，可以根据需要进行调整）
    point.positions[0] = 1.0;
    point.positions[1] = 0.5;
    point.positions[2] = 0.0;
    point.positions[3] = -0.5;
    point.positions[4] = 1.0;
    point.positions[5] = -1.0;

    // 设置时间
    point.time_from_start = ros::Duration(1.0);

    // 将点添加到轨迹中
    traj.points.push_back(point);

    // 发布消息
    while (ros::ok())
    {
        // 打印消息
        ROS_INFO("Publishing joint trajectory");
        
        // 设置当前时间戳
        traj.header.stamp = ros::Time::now();

        // 发布轨迹消息
        arm_pub.publish(traj);

        // 等待
        loop_rate.sleep();
    }

    return 0;
}
