#include<ros/ros.h>

#include "mission_msgs/RegisterMission.h"
#include "mission_msgs/MissionFeedback.h"

// #include<spinner.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"test_msg_queues");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient missionClient = nh.serviceClient<mission_msgs::RegisterMission>("mission_receiver/add_missions"); 
    ros::ServiceClient feedbackClient = nh.serviceClient<mission_msgs::MissionFeedback>("mission_receiver/get_feedbacks");

    mission_msgs::RegisterMission mSrv;
    mission_msgs::MissionMsg MissionMsg;

    int seq = 0;
    for(int i=0;i<10;i++)
    {
        MissionMsg.header.stamp = ros::Time::now();
        MissionMsg.mission_id = i;
        
        if(i%2==0)
        {
            MissionMsg.mission_type = 1; // command type
            if(i>5)
                MissionMsg.priority = mission_msgs::MissionMsg::HIGH_LEVEL_PRIORITY;
            else
                MissionMsg.priority = mission_msgs::MissionMsg::MEDIUM_LEVEL_PRIORITY;
        }
        else
        {
            MissionMsg.mission_type = 2; 
            MissionMsg.mission_seq  = seq;
            ++seq;
        }
        mSrv.request.missions.push_back(MissionMsg);
    }
    if(missionClient.call(mSrv))
    {
        ROS_INFO("mission sent");
        if(mSrv.response.ack)
            ROS_INFO("Mission registered.");
        else
            ROS_INFO("No ack");
    }
    else
    {
        ROS_INFO("Service call failed");
    }

}