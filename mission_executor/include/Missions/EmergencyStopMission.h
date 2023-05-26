#include "Missions/MissionBase.h"

namespace done
{

class EmergencyStopMission : public MissionBase
{
public:

    EmergencyStopMission(bool on);

    // Setting up subscriber for battery level
    void initROS();

    /////////// Behavior tree nodes  //////////////

    // Checking if emergency stop mission has been issued and halt other missions if so
    BT::NodeStatus isEmergencyStop(BT::TreeNode& self);

    // Cancel all move_base goals
    BT::NodeStatus cancelAll();

private:

    // Node handle
    ros::NodeHandle m_pNh;

    // Ros publisher for navigation canceling
    ros::Publisher m_cancelNav;

};

} // namespace done
