#include<ros/ros.h>
#include<gtest/gtest.h>

#include "mission_msgs/RegisterMission.h"
#include "mission_msgs/MissionFeedback.h"

#include "mission_receiver/missionQueues.hpp"
#include "mission_receiver/mission_receiver.hpp"

mission_msgs::MissionMsg createMissionMessage(uint64_t missionID, uint64_t type)
{
    mission_msgs::MissionMsg msg;
    msg.header.stamp = ros::Time::now();
    msg.priority = mission_msgs::MissionMsg::MEDIUM_LEVEL_PRIORITY;
    msg.mission_id = missionID;
    msg.mission_type = type;
    return msg;
}


TEST(APItests, MissionQueue)
{
    mission_manager::DoubleEndedQueue<mission_msgs::MissionMsg> queue;
    
    auto msg = createMissionMessage(101,mission_msgs::MissionMsg::MISSION);
    queue.enqueue(msg);
    auto msg2 = createMissionMessage(102,mission_msgs::MissionMsg::MISSION);
    queue.enqueue(msg2);
    ASSERT_TRUE(queue.deleteEntry(102));
    auto deq_result = queue.dequeue();
    ASSERT_TRUE(deq_result.has_value());
    
    ASSERT_EQ(msg, deq_result.value());

    // queue.stopQueue();
    // ASSERT_FALSE(queue.dequeue());
}


TEST(APItests, CommandQueue)
{
    mission_manager::PriorityBasedQueue queue;
    auto msg = createMissionMessage(101,mission_msgs::MissionMsg::COMMAND);
    queue.enqueue(msg);
    auto deq_result = queue.dequeue();
    ASSERT_TRUE(deq_result.has_value());
    
    ASSERT_EQ(msg, deq_result.value());

    // priority test
    auto msg_H = createMissionMessage(101,mission_msgs::MissionMsg::COMMAND);
    msg_H.priority = mission_msgs::MissionMsg::HIGH_LEVEL_PRIORITY;
    auto msg_M = createMissionMessage(102,mission_msgs::MissionMsg::COMMAND);
    msg_M.priority = mission_msgs::MissionMsg::MEDIUM_LEVEL_PRIORITY;
    auto msg_L = createMissionMessage(103,mission_msgs::MissionMsg::COMMAND);
    msg_L.priority = mission_msgs::MissionMsg::LOW_LEVEL_PRIORITY;

    queue.enqueue(msg_L);
    queue.enqueue(msg_M);
    queue.enqueue(msg_H);

    deq_result = queue.dequeue();
    ASSERT_TRUE(deq_result.has_value());
    ASSERT_EQ(msg_H, deq_result.value());

    deq_result = queue.dequeue();
    ASSERT_TRUE(deq_result.has_value());
    ASSERT_EQ(msg_M, deq_result.value());

    deq_result = queue.dequeue();
    ASSERT_TRUE(deq_result.has_value());
    ASSERT_EQ(msg_L, deq_result.value());

    // queue.stopQueue();
    // ASSERT_FALSE(queue.dequeue());
}

// TEST(NodeTest, MissionReceiverBasicFunctions)
// {
//     ros::NodeHandle pnh("~");
//     mission_manager::MissionReceiver mission_receiver(pnh);

//     mission_msgs::RegisterMissionRequest req; 
//     mission_msgs::RegisterMissionResponse res;

//     req.missions.push_back(createMissionMessage(101,1));
//     req.missions.push_back(createMissionMessage(102,1));
//     req.missions.push_back(createMissionMessage(103,1));

//     bool isSuccess = mission_receiver.receiveMissionCallback(req,res);
//     ASSERT_TRUE(isSuccess);
//     ASSERT_TRUE(res.ack);
// }

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
