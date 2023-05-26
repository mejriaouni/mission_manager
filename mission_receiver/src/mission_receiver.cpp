#include "mission_receiver/mission_receiver.hpp"
#include <map>
#include <nlohmann/json.hpp>
#include <vector>

namespace mission_manager{

typedef actionlib::SimpleActionClient<mission_msgs::DoneMissionAction> Client;

// tempfix_stop_command
typedef std::map<std::string, std::vector<int>> jobMapType;
jobMapType jobMap; // Map with string key and vector value, this should be moved to a hpp file instead

MissionReceiver::MissionReceiver(ros::NodeHandle& pnh, ros::NodeHandle& nh, std::string actionServerName):
    missionClient_(nh, actionServerName, true)
{
    receiveService_ = pnh.advertiseService("add_missions", 
                        &MissionReceiver::receiveMissionCallback, this);
    feedbackService_ = pnh.advertiseService("get_feedbacks", 
                        &MissionReceiver::sendFeedbackCallback, this);

    // tempfix_stop_command
    cancel_nav = pnh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);

    isEmergencyStop_    = false;
    isPause_            = false;
    readyForNextMission_= true;

    // start msg processing threads
    missionProcessingThread = std::thread(&MissionReceiver::processMissions, this);
    commandProcessingThread = std::thread(&MissionReceiver::processCommands, this);
}

MissionReceiver::~MissionReceiver()
{
    missionQ_.stopQueue();
    commandQ_.stopQueue();
    feedbackQ_.stopQueue();
    missionProcessingThread.join();
    commandProcessingThread.join();
}

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// The tests will not work, becasue the Request message have been changed
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
bool MissionReceiver::receiveMissionCallback(
                    mission_msgs::RegisterMissionOne::Request &req, mission_msgs::RegisterMissionOne::Response &res)
{
    // Create a Register Mission Message
    // mission_msgs::RegisterMission mSrv;
    mission_msgs::RegisterMission mSrv;

    // The RegisterMission.requst.missions expects a MissionMsg in the push_back function
    // So we have to copy the values from RegisterMissionOne into a MissionMsg
    mission_msgs::MissionMsg MissionMsg;

    MissionMsg.header = req.header;
    MissionMsg.mission_id = req.mission_id;
    MissionMsg.mission_type = req.mission_type;
    MissionMsg.priority = req.priority;
    MissionMsg.mission_seq = req.mission_seq;
    MissionMsg.mission_args = req.mission_args;

    // Put the mission in the array missions
    mSrv.request.missions.push_back(MissionMsg);

    // tempfix_stop_command
    if (req.mission_type == 5)
    {
        nlohmann::json json = json.parse(req.mission_args);

        std::vector<int> mission_id_vector;
        jobMap.insert(std::pair<std::string, std::vector<int>>(json["job_id"], mission_id_vector));
        
        if (jobMap.find(json["job_id"]) == jobMap.end()) {
          // not found
        } else {
          jobMap[json["job_id"]].push_back(req.mission_id);
        }

        // Debug print of the items in jobMap
        // Create an iterator of the Map
        // std::map<std::string, std::vector<int>>::iterator it = jobMap.begin();

        // while (it != jobMap.end())
        // {
        //   std::string job_id = it->first;
        //   std::vector<int> mission_ids = it->second;

        //   unsigned int vecSize = mission_ids.size();

        //    loop trough the jobMap and print the mission_ids
        //   for(unsigned int i = 0; i < vecSize; i++)
        //   {
        //     ROS_INFO_STREAM("" << mission_ids[i]);
        //   }

        //   it++;
        // }
    }

    // TODO: Check for duplicate msgs/ resend msgs with same mission id

    for (auto mission : mSrv.request.missions)
    {
        if(isCommand(mission.mission_type))
        {
            commandQ_.enqueue(mission);
            ROS_INFO("COMMAND RECEIVED.");
        }
        else
        {
            missionQ_.enqueue(mission);
            ROS_INFO("MISSION RECEIVED.");
        }
    }
    
    res.ack = true;
    
    return true;
}

bool MissionReceiver::sendFeedbackCallback(mission_msgs::MissionFeedbackRequest &req,
                                mission_msgs::MissionFeedbackResponse &res)
{
    if(!feedbackQ_.isStop)
    {
        feedbackQ_.dequeueAll(res.feedbacks);
        return true;
    }
    else
    {
        return false;
    }

}

void MissionReceiver::clientFeedbackCallback(const mission_msgs::DoneMissionFeedbackConstPtr &msg)
{
    feedbackQ_.enqueue(msg->feedback);
    if(msg->feedback.mission_id == mission_id_)
    {
        // SUCCESSFUL FAILED ABORTED
        if(msg->feedback.mission_status.find("ACTIVE")!= std::string::npos) 
        {
            missionStatus_ = MissionStatus::FINISHED;
        }
    } 
    return;
}

void MissionReceiver::clientDoneCallback(const actionlib::SimpleClientGoalState &state,
                            const mission_msgs::DoneMissionResultConstPtr &msg)
{
    if(!isPause_)
    {
        missionStatus_ = MissionStatus::FINISHED;
        readyForNextMission_ = true;
    }
}


void MissionReceiver::processMissions()
{
    ros::Rate rate(1);
    while(true)
    {   
        if(readyForNextMission_ && !(isEmergencyStop_))
        {   
            missionClient_.waitForServer();
            auto result = missionQ_.dequeue();
            if(result.has_value())
            {
                auto mission = result.value();
                sendGoal(mission);
                readyForNextMission_ = false;
                mission_id_ = mission.mission_id;
                ROS_INFO("MISSION sent to executor.");
            }
        }
        // To exit the thread and join main thread when queue is stopped.
        if(missionQ_.isStop)
        {
            break;
        }
        rate.sleep();
    }
}

void MissionReceiver::processCommands() // TODO: check server availability
{
    ros::Rate rate(1);
    while(true)
    {
        auto result = commandQ_.dequeue();
        if(result.has_value())
        {
            auto command = result.value();
            MissionType missionType = static_cast<MissionType>(command.mission_type);
            // TODO: add mission to abandon current mission.
            switch (missionType)
            {
            case MissionType::MISSION_STOP:
            {
                // tempfix_stop_command
                // All the code below in this case should be moved into a BTNode 
                // and treated like the ohter commands, by calling the sendCommand function
                // This is just a quick fix to get the stop command working
                missionStatus_ = MissionStatus::FINISHED;

                nlohmann::json json = json.parse(command.mission_args);

                ROS_INFO_STREAM("STOP" << json["job_id"]);

                unsigned int vecSize = jobMap[json["job_id"]].size();

                ROS_INFO_STREAM("job_id:" << json["job_id"] << ", missions: " << vecSize);

                for(unsigned int i = 0; i < vecSize; i++) // loop trough the mission_id in the job
                {
                  ROS_INFO_STREAM("mission_id: " << jobMap[json["job_id"]][i]);
                  deleteMission(jobMap[json["job_id"]][i]); // Delete mission, current mission do not get canceled
                }

                // create empty GoalID message
                actionlib_msgs::GoalID stopMsg; 
                // send the empty messge to /move_base/cancel topic to stop the current navigation
                // this do not cancel the uv-c disinfection routine we would need to use the BT for that
                cancel_nav.publish(stopMsg); 
            }
                break;
            case MissionType::EMERGENCYSTOP_ON :
            {
                missionStatus_ = MissionStatus::FINISHED;
                sendGoal(command);
                readyForNextMission_ = false;
                isEmergencyStop_     = true;  
                ROS_INFO("EMERGENCY STOP ON");               
            }
                break;
            case MissionType::EMERGENCYSTOP_OFF :
            {
                sendGoal(command);
                readyForNextMission_ = true;
                isEmergencyStop_     = false;
                ROS_INFO("EMERGENCY STOP OFF");
            }
                break;
            case MissionType::MISSION_PAUSE :
            {
                missionStatus_ = MissionStatus::PAUSED;
                if(!isEmergencyStop_)
                {
                    isPause_ = true;
                    sendGoal(command);
                    ROS_INFO("MISSION PAUSE SENT");
                }
            }
                break;
            case MissionType::MISSION_RESUME :
            {
                missionStatus_ = MissionStatus::ACTIVE;
                if(!isEmergencyStop_)
                {
                    sendGoal(command);
                    isPause_ = false;
                    ROS_INFO("MISSION RESUME SENT");
                }
            }
                break;
            default:
                break;
            }
        }
        if(commandQ_.isStop)
        {
            break; 
        }
        rate.sleep();
    }
}

bool MissionReceiver::deleteMission(uint64_t mission_id)
{
    return missionQ_.deleteEntry(mission_id);
}

bool MissionReceiver::isCommand(uint64_t mission_type)
{
    // TODO : Write unit test for this function, to check for newly added missions/commands
    MissionType missionType = static_cast<MissionType>(mission_type);
    if(missionType == MissionType::EMERGENCYSTOP_OFF ||
        missionType == MissionType::EMERGENCYSTOP_ON ||
        missionType == MissionType::MISSION_PAUSE ||
        missionType == MissionType::MISSION_RESUME ||
        missionType == MissionType::MISSION_STOP)
    {
        return true;
    }

    return false;
}

void MissionReceiver::sendGoal(mission_msgs::MissionMsg &msg)
{
    mission_msgs::DoneMissionGoal goal;
    goal.mission = msg;
    missionClient_.sendGoal(goal,
                boost::bind(&MissionReceiver::clientDoneCallback, this, _1, _2),
                Client::SimpleActiveCallback(),
                boost::bind(&MissionReceiver::clientFeedbackCallback, this, _1));
}

} // namespace
