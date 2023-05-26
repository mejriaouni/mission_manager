#include "Missions/MissionBase.h"
#include <std_msgs/Int16.h>

namespace done
{

class BatteryChargingMission : public MissionBase
{

public:

    // Possible tasks of this mission
    enum class Task
    {
        NAVIGATION = 0,
        CHARGING   = 1
    };

    struct ChargingStation
    {
        std::string stationName;
        geometry_msgs::PoseStamped stationPose;
    };

    BatteryChargingMission();

    // Function for setting mission up
    void init() override;

    // Get active task name as string
    std::string get_activeTask() override;

    // Setting up subscriber for battery level
    void initROS();

    // Get the name of task enum as string
    constexpr const char* taskToString(Task task);

    // Get charging and station data from yaml
    void readChargingStationsData();

    // Setting active task for this mission
    void set_activeTask(Task task);

    // ROS callback for setting battery level
    void chargeCallback(const std_msgs::Int16::ConstPtr& msg);

    /////////// Behavior tree nodes  //////////////

    // Navigation handling of this mission
    BT::NodeStatus navigate() override;

    // Mock action for charging battery
    BT::NodeStatus charge();

    // Behavior tree condition node checking battery charge level
    BT::NodeStatus isBatteryLow();

    // Check if battery is fully charged
    BT::NodeStatus isBatteryFull();


private:

    // Node handle
    ros::NodeHandle m_pNh;

    // Pose of charging station
    geometry_msgs::PoseStamped m_missionStartPose;

    // List of rooms with their names and poses
    std::vector<ChargingStation> m_stationList;

    // Battery charge level
    int m_chargeLvl;

    // Minimum charge level before going to charge
    int m_minChargeLvl;

    /// Battery charge subscriber
    ros::Subscriber m_chargeSub;

    /// Battery charge mission publisher
    ros::Publisher m_chargePub;

    // Currently active task
    Task m_activeTask;
};

} // namespace done

