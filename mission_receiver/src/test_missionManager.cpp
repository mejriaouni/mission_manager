#include<ros/ros.h>

#include <nlohmann/json.hpp>
#include <chrono>
#include <thread>

#include "mission_msgs/RegisterMission.h"
#include "mission_msgs/MissionFeedback.h"

// #include<spinner.h>

mission_msgs::MissionMsg createMission(uint64_t type, uint64_t priority)
{
    mission_msgs::MissionMsg msg;
    msg.header.stamp = ros::Time::now();

    msg.mission_type = type;
    msg.priority = priority;

    return msg;
}

void print_feedbacks(ros::ServiceClient& feedbackClient, mission_msgs::MissionFeedback& fbSrv)
{
        if(feedbackClient.call(fbSrv))
    {
        ROS_INFO("Feedback request sent");
    }
    else
    {
        ROS_INFO("Feedback call failed");
    }
}

void sendMissions(ros::ServiceClient& missionClient, mission_msgs::RegisterMission& mSrv)
{
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
        ROS_INFO("Mission service call failed");
        ROS_ASSERT(false);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"test_mission_manager");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient missionClient = nh.serviceClient<mission_msgs::RegisterMission>("mission_receiver/add_missions"); 
    ros::ServiceClient feedbackClient = nh.serviceClient<mission_msgs::MissionFeedback>("mission_receiver/get_feedbacks");

    mission_msgs::RegisterMission mSrv;
    mission_msgs::MissionFeedback fbSrv;
    mission_msgs::MissionMsg MissionMsg;

    ROS_INFO("Sending a Disinfect Mission");
    MissionMsg.header.stamp = ros::Time::now();
    MissionMsg.mission_id = 112;
    MissionMsg.mission_type = 5; // disinfect type
    MissionMsg.priority = mission_msgs::MissionMsg::MEDIUM_LEVEL_PRIORITY;
    nlohmann::json json;
    json["room_id"] = 0;
    MissionMsg.mission_args = json.dump();
    mSrv.request.missions.push_back(MissionMsg);
    sendMissions(missionClient, mSrv);

    // ============================================================//
    ROS_INFO("Waiting for 5 seconds for robot to start moving");
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    print_feedbacks(feedbackClient, fbSrv);

    mSrv.request.missions.clear();
    mSrv.response.ack = false;

    ROS_INFO("Sending a pause command");
    MissionMsg.header.stamp = ros::Time::now();
    MissionMsg.mission_id = 113;
    MissionMsg.mission_type = 3; // pause type
    MissionMsg.priority = mission_msgs::MissionMsg::HIGH_LEVEL_PRIORITY;
    MissionMsg.mission_args = "";
    mSrv.request.missions.push_back(MissionMsg);
    sendMissions(missionClient, mSrv);

    // ============================================================//
    ROS_INFO("Waiting for 5 seconds");
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    print_feedbacks(feedbackClient, fbSrv);

    mSrv.request.missions.clear();
    mSrv.response.ack = false;

    ROS_INFO("Sending a resume command");
    MissionMsg.header.stamp = ros::Time::now();
    MissionMsg.mission_id = 114;
    MissionMsg.mission_type = 4; // resume type
    MissionMsg.priority = mission_msgs::MissionMsg::HIGH_LEVEL_PRIORITY;
    MissionMsg.mission_args = "";
    mSrv.request.missions.push_back(MissionMsg);
    sendMissions(missionClient, mSrv);

    // ============================================================//
    ROS_INFO("Waiting for 3 seconds");
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    print_feedbacks(feedbackClient, fbSrv);

    mSrv.request.missions.clear();
    mSrv.response.ack = false;

    ROS_INFO("Sending a emergency stop command");
    MissionMsg.header.stamp = ros::Time::now();
    MissionMsg.mission_id = 115;
    MissionMsg.mission_type = 1; // emergency on type
    MissionMsg.priority = mission_msgs::MissionMsg::HIGH_LEVEL_PRIORITY;
    MissionMsg.mission_args = "";
    mSrv.request.missions.push_back(MissionMsg);
    sendMissions(missionClient, mSrv);

    // ============================================================//
    ROS_INFO("Waiting for 3 seconds");
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    print_feedbacks(feedbackClient, fbSrv);

    mSrv.request.missions.clear();
    mSrv.response.ack = false;

    ROS_INFO("Sending a emergency cancel command");
    MissionMsg.header.stamp = ros::Time::now();
    MissionMsg.mission_id = 116;
    MissionMsg.mission_type = 2; // emergency off type
    MissionMsg.priority = mission_msgs::MissionMsg::HIGH_LEVEL_PRIORITY;
    MissionMsg.mission_args = "";
    mSrv.request.missions.push_back(MissionMsg);
    sendMissions(missionClient, mSrv);

    // ============================================================//
    ROS_INFO("Waiting for 3 seconds");
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    print_feedbacks(feedbackClient, fbSrv);

    mSrv.request.missions.clear();
    mSrv.response.ack = false;
    json["room_id"] = 1;

    ROS_INFO("Sending a disinfect mission");
    MissionMsg.header.stamp = ros::Time::now();
    MissionMsg.mission_id = 115;
    MissionMsg.mission_type = 5; // disinfect type
    MissionMsg.priority = mission_msgs::MissionMsg::MEDIUM_LEVEL_PRIORITY;
    MissionMsg.mission_args = json.dump();
    mSrv.request.missions.push_back(MissionMsg);
    sendMissions(missionClient, mSrv);
}