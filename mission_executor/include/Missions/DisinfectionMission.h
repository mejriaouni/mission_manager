#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "Missions/MissionBase.h"

namespace done
{

    class DisinfectionMission : public MissionBase
    {

    public:
        bool m_goalReached = false;
        // Possible tasks for this mission
        enum class Task
        {
            NAVIGATION = 0,
            LAMP_HEATUP = 1,
            DISINFECTING_ROOM = 2
        };

        struct Room
        {
            std::string roomName;
            geometry_msgs::PoseStamped roomPose;
            geometry_msgs::PoseStamped roomwaitPose;
            std::vector<geometry_msgs::PoseStamped> disinfectionPoses;
            std::string jobid;
            std::string part;
            int dose;
        };

        DisinfectionMission();
        static BT::PortsList providedPorts()
        {
            return {BT::OutputPort<std::vector<geometry_msgs::PoseStamped>>("current_room_disinfection_pose")};
        }

        // Function for setting mission up
        void init() override;
        bool getUserConfirmation(ros::NodeHandle &nh, const std::string &job_id, const std::string &robot_id, const std::string &action_id, const std::string &action_type);

        // get the movebase status whether the goal is accepted , reached ...
        void updateGoalStatus();

        // Get the name of task enum as string
        std::string taskToString(Task task);

        // Setting active task for this mission
        void set_activeTask(Task task);

        // Get active task name as string
        std::string get_activeTask() override;

        // Read mission specific data from message
        void readSpecificData() override;

        // Load room start disinfection positions from yaml
        void readRoomData(std::string file);

        // Read JSON data to Room with pose
        void setRoomData(nlohmann::json data);

        /////////// Behavior tree nodes  //////////////

        // Navigation handling for this mission
        BT::NodeStatus navigate() override;

        // Behavior tree mock action node
        BT::NodeStatus getDisinfectionPath(BT::TreeNode &self);

        // Behavior tree mock action node
        BT::NodeStatus warmUpLamps();

        // Behavior tree mock action node
        BT::NodeStatus turnOffLamps();

        // Behavior tree mock action node
        BT::NodeStatus isClearToDisinfect();

        // Behavior tree mock condition node
        BT::NodeStatus isRoomPositionCorrect();

        // Behavior tree node for checking disinfection flag
        BT::NodeStatus isDisinfectionFinished(BT::TreeNode &self);

    private:
        // Node handle
        ros::NodeHandle m_pNh;

        // Id of the room to disinfect
        int m_roomId;
        ros::Publisher room_pub;
        // Name of the room to disinfect
        std::string m_roomName;

        // Room with name and poses used in the mission
        Room current_room;

        // Was navigation completed for this mission, will need to do calculations in the future
        bool m_isRoomCorrect;

        // Currently active task
        Task m_activeTask;
    };

} // namespace done
