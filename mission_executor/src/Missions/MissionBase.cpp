#include "Missions/MissionBase.h"

#include "mission_msgs/MissionMsg.h"

namespace done
{

using namespace BT;

MissionBase::MissionBase()
: m_pNh("~")
, m_missionType(Mission::NOMISSION)
, m_priority(0)
{}

void MissionBase::readSpecificData(){};

void MissionBase::init(){};

NodeStatus MissionBase::navigate(){return NodeStatus::SUCCESS;};

void MissionBase::readCommonData(mission_msgs::MissionMsg mission)
{
    m_missionId   = mission.mission_id;
    m_missionType = Mission(mission.mission_type);
    m_priority    = mission.mission_id;
    m_missionSeq  = mission.mission_seq;
    m_missionArgs = mission.mission_args;
    std::string mission_id_str = std::to_string(m_missionId);
    ros::param::set("/mission_id", mission_id_str);

}



}//namespace done

