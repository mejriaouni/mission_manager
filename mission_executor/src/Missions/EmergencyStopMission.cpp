#include "Missions/EmergencyStopMission.h"

namespace done
{

using namespace BT;

EmergencyStopMission::EmergencyStopMission(bool on)
: m_pNh("~")
{
    initROS();
    if (on)
    {
        set_type(Mission::EMERGENCYSTOP_ON);
    }
    else
    {
        set_type(Mission::EMERGENCYSTOP_OFF);
    }
}

// Setting up subscriber for battery level
void EmergencyStopMission::initROS()
{
    m_cancelNav  = m_pNh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
}

// Checking if emergency stop mission has been issued and halt other missions if so
NodeStatus EmergencyStopMission::isEmergencyStop(TreeNode& self)
{
    Optional<std::string> msg = self.getInput<std::string>("activeMission");
    if (!msg)
    {
        return NodeStatus::FAILURE;
    }
    if (msg.value()==std::to_string((int)Mission::EMERGENCYSTOP_ON))
    {
        ROS_INFO_STREAM("Sending cancel command to navigation.");
        this->cancelAll();
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}

// Cancel all move_base goals
NodeStatus EmergencyStopMission::cancelAll()
{
    actionlib_msgs::GoalID msg;
    m_cancelNav.publish(msg);
    return NodeStatus::SUCCESS;
}



} // namespace done
