#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include "Missions/MissionBase.h"
#include "Missions/DisinfectionMission.h"
#include "Missions/BatteryChargingMission.h"
#include "Missions/EmergencyStopMission.h"
#include "Missions/MissionContainer.h"

#include "mission_msgs/MissionMsg.h"
#include "mission_msgs/FeedbackMsg.h"
#include "mission_msgs/MissionFeedback.h"
#include "mission_msgs/DoneMissionAction.h"
#include "std_msgs/String.h"

namespace done
{

class MissionInterface
{

public:

    MissionInterface();

    // Behavior tree condition node to check if there are active missions in queue
    BT::NodeStatus isMissionActive(BT::TreeNode& self);

    // Called after mission ends
    void missionEnd(MissionBase::MissionStatus status);

    // ROS callback for action server
    void missionActionCallback(const mission_msgs::DoneMissionGoal::ConstPtr &msg);

    // Set the status for current mission
    void setMissionStatus(MissionBase::MissionStatus status);

    // Convert enum value to string representation
    std::string getStatusString(MissionBase::MissionStatus status);

    // All available missions in a container
    MissionContainer m_missionContainer;

private:

    // Node handle
    ros::NodeHandle m_pNh;

    // Is waiting for mission started
    bool m_idleStateStart;

    // Emergency stop on/off flag
    bool m_isEmergencyStop;

    // Active mission
    std::shared_ptr<MissionBase> m_activeMission;

    // Action server/client
    actionlib::SimpleActionServer<mission_msgs::DoneMissionAction> m_actionServer;
    actionlib::SimpleActionClient<mission_msgs::DoneMissionAction> m_actionClient;

    // Create messages that are used to published feedback/result for action client
    mission_msgs::DoneMissionFeedback m_feedback;
    mission_msgs::DoneMissionResult m_result;

    // Saving mission info
    mission_msgs::MissionMsg m_lastMissionInfo;
    Mission m_lastMissionType;

};
}//namespace done
