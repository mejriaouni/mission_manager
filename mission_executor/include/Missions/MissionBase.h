#ifndef _MISSIONBASE_H_
#define _MISSIONBASE_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nlohmann/json.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>

#include "BTNodes/NavigationInterface.h"
#include "Missions/MissionEnum.h"

#include "mission_msgs/MissionMsg.h"

namespace done
{

class MissionBase
{

public:

    enum class Task{};

    enum class MissionStatus
    {
        RUNNING = 1,
        SUCCESSFUL = 2,
        FAILED = 3,
        ABORTED =4,
    };

    MissionBase();

    constexpr const char* taskToString(Task task){return "";};

    void virtual readSpecificData();

    void virtual init();

    void virtual set_activeTask(Task task){};

    std::string virtual get_activeTask(){return "";};

    BT::NodeStatus virtual navigate();

    void readCommonData(mission_msgs::MissionMsg mission);

    // Getters
    int get_id(){ return m_missionId; }

    Mission get_type(){ return m_missionType; }

    std::string getMissionArguments(){ return m_missionArgs; }

    MissionStatus get_status(){ return m_status; }

    int get_priority(){ return m_priority; }

    // Setters
    void set_priority(int priority){ m_priority = priority; }

    void set_status(MissionStatus newStatus){ m_status = newStatus; }

    void set_id(int id){ m_missionId = id; }

    void set_type(Mission type){ m_missionType = type; }

    //Navigation
    NavigationInterface navigation;

private:

    // Node handle
    ros::NodeHandle m_pNh;

    ///Mission common data
    int m_missionId;
    Mission m_missionType;
    int m_priority;
    int m_missionSeq;
    std::string m_missionArgs;

    // Mission status
    MissionStatus m_status;

};
}//namespace done

#endif
