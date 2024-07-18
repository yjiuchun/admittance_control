#include "ur5_communication.hpp"
#include <iostream>
#include <ur5_control/ur5_pos.h>


sensor_msgs::JointState::ConstPtr state;
geometry_msgs::WrenchStamped::ConstPtr force;
std_msgs::Float64MultiArray target_pos;
std_msgs::Float64MultiArray current_pos;

ur5_control::ur5_pos ur5_pos_msg;
bool ifnewcommand = false;

ros::Time current_time;

void poscommandcallback(const ur5_control::ur5_pos::ConstPtr& msg)
{
  ROS_INFO("Received UR5 positions command: arm1: %f, arm2: %f, arm3: %f, arm4: %f, arm5: %f, arm6: %f",
           msg->arm1, msg->arm2, msg->arm3, msg->arm4, msg->arm5, msg->arm6);
    ur5_pos_msg = *msg;
    ifnewcommand = true;
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "RosCommunication");
    ros::NodeHandle nh;

    std::string JointStateTopic = "/joint_states";
    std::string JointCommandTopic = "/arm_group_controller/command";
    std::string FtSensorTopic = "/ft_sensor_topic";
    RosCommunication RC(JointCommandTopic, JointStateTopic, FtSensorTopic, nh);
    ros::Subscriber sub = nh.subscribe("ur5_pos", 1000, poscommandcallback);
    target_pos.data.push_back(0.0);
    target_pos.data.push_back(0.0);
    target_pos.data.push_back(0.0);
    target_pos.data.push_back(0.0);
    target_pos.data.push_back(0.0);
    target_pos.data.push_back(0.0);
    current_pos.data.push_back(0.0);
    current_pos.data.push_back(0.0);
    current_pos.data.push_back(0.0);
    current_pos.data.push_back(0.0);
    current_pos.data.push_back(0.0);
    current_pos.data.push_back(0.0);
    ros::Rate loop_rate(10); // 设置循环的频率，这里是10Hz
    while(ros::ok())
    {   
        ros::spinOnce();
        if(ifnewcommand)
        {
        state = RC.GetJointState();
        int i = 0;
        for (double joint : state->position)
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
            target_pos.data[0] = ur5_pos_msg.arm1;
            target_pos.data[1] = ur5_pos_msg.arm2;
            target_pos.data[2] = ur5_pos_msg.arm3;
            target_pos.data[3] = ur5_pos_msg.arm4;
            target_pos.data[4] = ur5_pos_msg.arm5;
            target_pos.data[5] = ur5_pos_msg.arm6;

            RC.AddTrajPoint(2,current_pos,target_pos,10);
            ifnewcommand = false;
        }
        loop_rate.sleep();
    }
    return 0;
}