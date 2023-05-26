#include "mission_msgs/Room.h"

namespace done
{

    // SyncActionNode (synchronous action) with an input port.
    class FollowDisinfectionPath : public BT::CoroActionNode
    {

    public:
        FollowDisinfectionPath(const std::string &name, const BT::NodeConfiguration &config);

       /* static BT::PortsList providedPorts()
        {
            return {BT::OutputPort<std::string>("isDisinfectionFinished")};
        }*/
        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<std::vector<geometry_msgs::PoseStamped>>("current_room_disinfection_pose")};
        }
        void currentRoomCallback(const mission_msgs::Room::ConstPtr &msg);

        // Behavior tree node for mock disinfection action
        BT::NodeStatus tick() override;

        // Cleaning up disinfection data if halt was issued
        void cleanup(bool halted);

        // Mission halt implementation
        void halt() override;

    private:
        // Disinfection progress variable
        int m_iterations;
        int Number_of_disinfection_points;
        
    };

} // namespace done
