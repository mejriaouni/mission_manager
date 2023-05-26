#include "BTNodes/MissionInterface.h"
#include "std_msgs/String.h"
#include "ros/ros.h"


namespace done
{

const std::string DISINFECTION      = "Disinfection";
const std::string EMERGENCYSTOP_ON  = "EmergencyStopOn";
const std::string EMERGENCYSTOP_OFF = "EmergencyStopOff";
const std::string CHARGING          = "Charging";

const std::string ACTION_SERVER_NAME = "mission_action_server";


using namespace BT;

MissionInterface::MissionInterface()
: m_pNh("~")
, m_actionServer(m_pNh, ACTION_SERVER_NAME, boost::bind(&MissionInterface::missionActionCallback, this, _1), false)
, m_actionClient(m_pNh, ACTION_SERVER_NAME, true)
, m_idleStateStart(true)
, m_activeMission(nullptr)
, m_isEmergencyStop(false)
, m_lastMissionType(Mission::NOMISSION)

{
    // All missions have to be initialized from start for behavior tree nodes to be able to react
    m_missionContainer.addData(DISINFECTION,   std::make_shared<DisinfectionMission>());
    m_missionContainer.addData(CHARGING,    std::make_shared<BatteryChargingMission>());
    m_missionContainer.addData(EMERGENCYSTOP_ON, std::make_shared<EmergencyStopMission>(true));
    m_missionContainer.addData(EMERGENCYSTOP_OFF, std::make_shared<EmergencyStopMission>(false));

    // Start mission action server
    m_actionServer.start();
}

// Behavior tree condition node to check if there are active missions in queue
NodeStatus MissionInterface::isMissionActive(TreeNode& self)
{
    if(m_idleStateStart)
    {
        ROS_INFO_STREAM("--------------------------");
        ROS_INFO_STREAM("Waiting for mission");
        m_idleStateStart = false;
    }
    // If mission is active
    if (m_activeMission && m_activeMission->get_status()==MissionBase::MissionStatus::RUNNING)
    {
        self.setOutput("activeMission", std::to_string((int)m_activeMission->get_type()));
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}

// Called after mission ends
void MissionInterface::missionEnd(MissionBase::MissionStatus status)
{
    ROS_INFO_STREAM("Mission of type: " << (int)m_activeMission->get_type() << " ended with status: " << getStatusString(status) );
    m_activeMission->navigation.setIdle();
    m_activeMission->set_status(status);
    m_activeMission = nullptr;
    m_idleStateStart = true;
}


// ROS callback for setting new missions
void MissionInterface::missionActionCallback(const mission_msgs::DoneMissionGoal::ConstPtr &msg )
{
    ros::Rate rate(1);
    mission_msgs::MissionMsg mission = msg->mission;
    Mission missionType = static_cast<Mission>(mission.mission_type);

    // Ignore mission request if emergency stop is active (unless its emergencystop on/off request itself)
    if (m_isEmergencyStop && missionType!=Mission::EMERGENCYSTOP_ON && missionType!=Mission::EMERGENCYSTOP_OFF)
    {
        ROS_INFO_STREAM("'Emergency stop' active. Ignoring request");
        m_feedback.feedback.header.stamp = ros::Time::now();
        m_feedback.feedback.mission_status = "'Emergency stop' active. Ignoring request";
        m_actionServer.publishFeedback(m_feedback);
        m_actionServer.setPreempted();
        return;
    }

    // Check for resume - rerun last mission
    if (missionType==Mission::MISSION_RESUME)
    {
        if (m_lastMissionType==Mission::NOMISSION)
        {
            ROS_INFO_STREAM("No mission was paused previously.");
            m_actionServer.setPreempted();
            return;
        }
        ROS_INFO_STREAM("Resuming last executed mission");
        mission = m_lastMissionInfo;
        missionType = m_lastMissionType;
    }

    switch (missionType)
    {
        case Mission::NOMISSION:
            m_actionServer.setPreempted();
            m_lastMissionType = Mission::NOMISSION;
            return;
            break;

        case Mission::EMERGENCYSTOP_ON:
            if (m_isEmergencyStop)
            {
                ROS_INFO_STREAM("Mission 'Emergency stop' already active");
                m_feedback.feedback.header.stamp = ros::Time::now();
                m_feedback.feedback.mission_status = "Mission 'Emergency stop' already active";
                m_actionServer.publishFeedback(m_feedback);
                m_actionServer.setPreempted();
                return;
            }
            ROS_INFO_STREAM("Mission 'Emergency stop' activated");
            m_isEmergencyStop = true;
            m_activeMission = m_missionContainer.getData<std::shared_ptr<EmergencyStopMission>>(EMERGENCYSTOP_ON);
            m_lastMissionType = Mission::NOMISSION;
            break;

        case Mission::EMERGENCYSTOP_OFF:
            if (!m_isEmergencyStop)
            {
                ROS_INFO_STREAM("Mission 'Emergency stop' already inactive");
                m_feedback.feedback.header.stamp = ros::Time::now();
                m_feedback.feedback.mission_status = "Mission 'Emergency stop' already inactive";
                m_actionServer.publishFeedback(m_feedback);
                m_actionServer.setPreempted();
                return;
            }
            ROS_INFO_STREAM("Mission 'Emergency stop' deactivated");
            m_activeMission = m_missionContainer.getData<std::shared_ptr<EmergencyStopMission>>(EMERGENCYSTOP_OFF);
            m_isEmergencyStop = false;
            m_lastMissionType = Mission::NOMISSION;
            break;

        case Mission::MISSION_PAUSE:
            if (m_lastMissionType==Mission::NOMISSION)
            {
                ROS_INFO_STREAM("No running mission to pause.");
                m_actionServer.setPreempted();
                return;
            }
            ROS_INFO_STREAM("Pausing current mission");
            m_actionServer.setPreempted();
            return;
            break;

        case Mission::DISINFECTROOM:
            ROS_INFO_STREAM("Mission 'Disinfect room activated");
            m_activeMission = m_missionContainer.getData<std::shared_ptr<DisinfectionMission>>(DISINFECTION);
            // Saving mission info for possible pausing
            m_lastMissionInfo = mission;
            m_lastMissionType = Mission::DISINFECTROOM;
            break;

        case Mission::BATTERYCHARGING:
            ROS_INFO_STREAM("Mission 'Battery charge' activated");
            m_activeMission = m_missionContainer.getData<std::shared_ptr<BatteryChargingMission>>(CHARGING);
            // Saving mission info for possible pausing
            m_lastMissionInfo = mission;
            m_lastMissionType = Mission::BATTERYCHARGING;
            break;

        default:
            break;
    }
    m_activeMission->init();
    m_activeMission->readCommonData(mission);
    m_activeMission->readSpecificData();

    // Set mission status to running for missions with something more to do
    // If it's EMERGENCYSTOP_OFF - flag has already been set and mission is finished.
    if (missionType!=Mission::EMERGENCYSTOP_OFF)
    {
        m_activeMission->set_status(MissionBase::MissionStatus::RUNNING);
    }
    else
    {
        m_activeMission->set_status(MissionBase::MissionStatus::SUCCESSFUL);
    }

    // Action server loop publishing feedback and listening for canceling
    m_feedback.feedback.mission_type = mission.mission_type;
    while (m_activeMission->get_status()==MissionBase::MissionStatus::RUNNING)
    {
        // Was cancel called ?
        if ( m_actionServer.isPreemptRequested() || !ros::ok()){
                m_actionServer.setPreempted();
                missionEnd(MissionBase::MissionStatus::ABORTED);
                return;
        }
        m_feedback.feedback.header.stamp = ros::Time::now();
        std::string roomid;
        ros::param::get("/roomid", roomid);

        std::string dose;
        ros::param::get("/roomdose", dose);

        std::string mission_id;
        ros::param::get("/mission_id", mission_id);

        m_feedback.feedback.mission_id = stoi(mission_id);
        std::string jobid;
        ros::param::get("/jobid", jobid); 

        std::string part;
        ros::param::get("/part", part);

        m_feedback.feedback.mission_status =
            "{\"mission_status\":\"" + getStatusString(m_activeMission->get_status()) + "\"," + "\"active_task\":\"" + m_activeMission->get_activeTask() + "\"," + "\"room_id\":\"" + roomid + "\"," + "\"job_id\":\"" + jobid + "\"," + "\"dose\":\"" + dose + "\"," + "\"part\":\"" + part + "\"}" ;
        m_actionServer.publishFeedback(m_feedback);

        rate.sleep();
    }

    // Publish last feedback after mission
    m_feedback.feedback.header.stamp = ros::Time::now();
         std::string roomid;
        ros::param::get("/roomid", roomid);

        std::string mission_id;
        ros::param::get("/mission_id", mission_id);     
        m_feedback.feedback.mission_id = stoi(mission_id);

        std::string jobid;
        ros::param::get("/jobid", jobid);     

        std::string part;
        ros::param::get("/part", part);
    m_feedback.feedback.mission_status =
            "{\"mission_status\":\"" + getStatusString(m_activeMission->get_status()) + "\"," + "\"active_task\":\"" + m_activeMission->get_activeTask() + "\"," + "\"room_id\":\"" + roomid + "\"," + "\"job_id\":\"" + jobid + "\"," + "\"part\":\"" + part + "\"}" ;
    m_actionServer.publishFeedback(m_feedback);

    // Publishing action server result
    switch (m_activeMission->get_status())
    {
        case MissionBase::MissionStatus::FAILED:
            m_result.result = 0;
            m_actionServer.setSucceeded(m_result);
            missionEnd(MissionBase::MissionStatus::FAILED);
            break;

        case MissionBase::MissionStatus::SUCCESSFUL:
            m_result.result = 1;
            m_actionServer.setSucceeded(m_result);
            missionEnd(MissionBase::MissionStatus::SUCCESSFUL);
            break;

        default:
            break;
    }
}

// Setting status for active mission
void MissionInterface::setMissionStatus(MissionBase::MissionStatus status)
{
    m_activeMission->set_status(status);
}

// Convert enum to string
std::string MissionInterface::getStatusString(MissionBase::MissionStatus status)
{
    switch (status)
    {
        case MissionBase::MissionStatus::RUNNING:
            return "Running";
            break;

        case MissionBase::MissionStatus::SUCCESSFUL:
            return "Successful";
            break;

        case MissionBase::MissionStatus::FAILED:
            return "Failed";
            break;

        case MissionBase::MissionStatus::ABORTED:
            return "Aborted";
            break;

        default:
            break;
    }
    return "unknown";
}

}//namespace done
