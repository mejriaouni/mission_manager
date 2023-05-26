#include <ros/package.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "Missions/BatteryChargingMission.h"

namespace done
{

using namespace BT;
using namespace std::chrono_literals;

BatteryChargingMission::BatteryChargingMission()
: m_pNh("~")
, m_chargeLvl(100)
{
    set_type(Mission::BATTERYCHARGING);
    init();
}

// Get the name of task enum as string
constexpr const char* BatteryChargingMission::taskToString(Task task)
{
    switch (task)
    {
        case Task::NAVIGATION: return "Navigation";
        case Task::CHARGING:   return "Charging";
        default: return "Unknown";
    }
}

// Get charging and station data from yaml
void BatteryChargingMission::readChargingStationsData()
{
    m_pNh.getParam("/charging_level", m_minChargeLvl);

    ChargingStation station;

    XmlRpc::XmlRpcValue stations;
    m_pNh.getParam("/charging_station_poses", stations);
    if (stations.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
        for(std::map<std::string,XmlRpc::XmlRpcValue>::iterator itrStations = stations.begin(); itrStations != stations.end(); ++itrStations )
        {
            station.stationName = itrStations->first;
            XmlRpc::XmlRpcValue poseStruct = itrStations->second;
            if(poseStruct.getType() == XmlRpc::XmlRpcValue::TypeStruct)
            {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id="map";
                for(std::map<std::string,XmlRpc::XmlRpcValue>::iterator itrAttributes = poseStruct.begin(); itrAttributes != poseStruct.end(); ++itrAttributes )
                {
                    XmlRpc::XmlRpcValue attribute = itrAttributes->first;
                    XmlRpc::XmlRpcValue attributes = itrAttributes->second;
                    for(std::map<std::string,XmlRpc::XmlRpcValue>::iterator itrXYZ = attributes.begin(); itrXYZ != attributes.end(); ++itrXYZ )
                    {
                        XmlRpc::XmlRpcValue xyzName = itrXYZ->first;
                        XmlRpc::XmlRpcValue xyzValue = itrXYZ->second;
                        if (xyzName=="w")
                        {
                            pose.pose.orientation.w = xyzValue;
                        }
                        if (xyzName=="x")
                        {
                            if (attribute=="orientation")
                            {
                                pose.pose.orientation.x = xyzValue;
                            }
                            else
                            {
                                pose.pose.position.x = xyzValue;
                            }

                        }
                        if (xyzName=="y")
                        {
                            if (attribute=="orientation")
                            {
                                pose.pose.orientation.y = xyzValue;
                            }
                            else
                            {
                                pose.pose.position.y = xyzValue;
                            }
                        }
                        if (xyzName=="z")
                        {
                            if (attribute=="orientation")
                            {
                                pose.pose.orientation.z = xyzValue;
                            }
                            else
                            {
                                pose.pose.position.z = xyzValue;
                            }
                        }
                    }

                }
                station.stationPose = pose;
                m_stationList.push_back(station);
            }
        }
    }
}

void BatteryChargingMission::init()
{
    initROS();
    readChargingStationsData();
}

NodeStatus BatteryChargingMission::navigate()
{
    NodeStatus navStatus = navigation.get_status();
    switch (navStatus)
    {
        case NodeStatus::SUCCESS:
            navigation.setIdle();
            break;

        case NodeStatus::FAILURE:
            // if navigation fails - mission fails
            this->set_status(MissionStatus::FAILED);
            navigation.setIdle();
            break;

        case NodeStatus::IDLE:
            ROS_INFO_STREAM("Starting navigation to charging station.");
            set_activeTask(Task::NAVIGATION);

            // create unique goalId
            navigation.setGoalId("charge"+std::to_string(ros::Time::now().toSec()));

            // TODO start pose has to be defined somehow (like closest, defined in mission cal, etc.)
            // for now, its just the first entry from yaml file
            m_missionStartPose = m_stationList[0].stationPose;

            navigation.goTo(m_missionStartPose);
            navStatus = NodeStatus::RUNNING;
            break;

        default:
            break;
    }
    return navStatus;
}

// Mock action for charging battery
NodeStatus BatteryChargingMission::charge()
{
    set_activeTask(Task::CHARGING);
    ROS_INFO_STREAM("Charging robot...");
    std::this_thread::sleep_for(5000ms);
    ROS_INFO_STREAM("Charging complete");
    m_chargeLvl = 100;
    navigation.setIdle();
    return NodeStatus::SUCCESS;
}

// Behavior tree condition node checking battery charge level
NodeStatus BatteryChargingMission::isBatteryLow()
{
    if (m_chargeLvl < m_minChargeLvl )
    {
        // Getting the pose for the only charging station
        m_missionStartPose = m_stationList[0].stationPose;
        return NodeStatus::FAILURE;
    }
    return NodeStatus::SUCCESS;
}

// Check if battery is fully charged
NodeStatus BatteryChargingMission::isBatteryFull()
{
    if (m_chargeLvl==100)
    {
        ROS_INFO_STREAM("Battery is fully charged");
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}

// ROS callback for setting battery level
void BatteryChargingMission::chargeCallback(const std_msgs::Int16::ConstPtr& msg)
{
    m_chargeLvl = msg->data;
    mission_msgs::MissionMsg message;
    message.mission_type = 3;
    message.priority = message.MEDIUM_LEVEL_PRIORITY;
    m_chargePub.publish(message);
}

// Setting up subscriber for battery level
void BatteryChargingMission::initROS()
{
    m_chargeSub  = m_pNh.subscribe("/charge_level", 1000, &BatteryChargingMission::chargeCallback, this);
    m_chargePub = m_pNh.advertise<mission_msgs::MissionMsg>("mission_receiver", 1, true);
}

// Set the active task
void BatteryChargingMission::set_activeTask(Task task)
{
     m_activeTask = task;
}

// Get std::string representation of currently active mission name
std::string BatteryChargingMission::get_activeTask()
{
    return taskToString(m_activeTask);
}

} // namespace done

