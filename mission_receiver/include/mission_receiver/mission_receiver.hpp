#pragma once

#include <thread>
#include <string>
#include <boost/bind.hpp>

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "mission_msgs/RegisterMission.h"
#include "mission_msgs/RegisterMissionOne.h"
#include "mission_msgs/MissionFeedback.h"
#include "mission_msgs/DoneMissionAction.h"
#include "mission_msgs/constants.hpp"
// #include "mission_msgs/DoneMissionActionResult.h"

#include "mission_receiver/missionQueues.hpp"

namespace mission_manager{

enum class MissionStatus{
    FINISHED    = 100,
    PAUSED      = 101,
    ACTIVE      = 102
};

class MissionReceiver{

public:

    /**
     * @brief Construct a new Mission Receiver:: Mission Receiver object
     * 
     * @param pnh ROS private node handle
     */
    MissionReceiver(ros::NodeHandle& pnh, ros::NodeHandle& nh, std::string actionServerName);

    /**
     * @brief Destroy the Mission Receiver:: Mission Receiver object
     * 
     */
    ~MissionReceiver();

    /**
     * @brief Callback function for the ROS service to send missions to robot
     * 
     * @param req   Request message of the service. Contains an array of mission messages
     * @param res   Response message of the service. Contains a boolean for acknowledgement
     * @return true : when service succeeded, and the response object has been filled with data
     * @return false: when call has failed and the response object will not be sent 
     */    
    bool receiveMissionCallback(mission_msgs::RegisterMissionOne::Request &req,
                                mission_msgs::RegisterMissionOne::Response &res);
    
    /**
     * @brief Callback function for the ROS service to send feedbacks to web-apps
     * 
     * @param req   Request message of the service. Empty message
     * @param res   Response message of the service. Contains an array of feedback messages
     * @return true : when service succeeded, and the response object has been filled with data
     * @return false: when call has failed and the response object will not be sent 
     */
    bool sendFeedbackCallback(mission_msgs::MissionFeedbackRequest &req,
                                mission_msgs::MissionFeedbackResponse &res);
    
    /**
     * @brief   Function to dequeue mission messages and call the corresponding executioner.
     * 
     */
    void processMissions();

    /**
     * @brief   Function to dequeue command messages and execute them.
     * 
     */
    void processCommands();

    /**
     * @brief   Function to receive feedback msgs from action client and put to feedback queue
     * 
     */
    void clientFeedbackCallback(const mission_msgs::DoneMissionFeedbackConstPtr &msg);

    void clientDoneCallback(const actionlib::SimpleClientGoalState &state,
                            const mission_msgs::DoneMissionResultConstPtr &msg);

    /**
     * @brief   Function to delete specific mission messages from the queue.
     * 
     * @param mission_id id of the message to be deleted.
     * @return true     Delete operation successful
     * @return false    Delete operation failed
     */
    bool deleteMission(uint64_t mission_id);

    /**
     * @brief Check whether a message is command or not.
     * 
     * @param mission_type 
     * @return true 
     * @return false 
     */
    bool isCommand(uint64_t mission_type);

    /**
     * @brief sends the list of missions in the queue using feedback message
     * 
     */
    void readMissionQueue();

    /**
     * @brief 
     * 
     * @param mission 
     */
    void sendGoal(mission_msgs::MissionMsg &msg);

private:
    ros::ServiceServer receiveService_;
    ros::ServiceServer feedbackService_;

    // tempfix_stop_command
    ros::Publisher cancel_nav; // temporary fix for the stop command, this Publisher is used to publish a message to /move_base/cancel

    // message queues
    DoubleEndedQueue<mission_msgs::MissionMsg> missionQ_;
    DoubleEndedQueue<mission_msgs::FeedbackMsg> feedbackQ_;
    PriorityBasedQueue commandQ_;

    // threads
    std::thread missionProcessingThread;
    std::thread commandProcessingThread;

    // Action client to call mission executer.
    actionlib::SimpleActionClient<mission_msgs::DoneMissionAction> missionClient_;

    // blockers
    std::atomic<bool> isEmergencyStop_;
    std::atomic<bool> isPause_;
    std::atomic<bool> readyForNextMission_;

    // current mission details
    std::atomic<uint64_t> mission_id_;
    std::atomic<MissionStatus> missionStatus_;
};


} // namespace
