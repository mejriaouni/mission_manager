#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include "BTNodes/FollowDisinfectionPath.h"
#include <nlohmann/json.hpp>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

namespace done
{

    using namespace BT;
    using namespace std::chrono_literals;
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    FollowDisinfectionPath::FollowDisinfectionPath(const std::string &name, const NodeConfiguration &config)
        : CoroActionNode(name, config), m_iterations(1)
    {
    }

    // Behavior tree node for mock disinfection action
    NodeStatus FollowDisinfectionPath::tick()
    {

        std::vector<geometry_msgs::PoseStamped> current_room_disinfection_pose;
        BT::TreeNode::getInput("current_room_disinfection_pose", current_room_disinfection_pose);
        if (current_room_disinfection_pose.empty())
        {
            ROS_ERROR_STREAM("Failed to get input: current_room_disinfection_pose");
            return NodeStatus::FAILURE;
        }


        MoveBaseClient ac("move_base", true);
        ac.waitForServer();

        for (size_t i = 0; i < current_room_disinfection_pose.size(); ++i)
        {
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose = current_room_disinfection_pose[i];

            ROS_INFO_STREAM("Sending goal " << (i + 1) << "/" << current_room_disinfection_pose.size() << " to move_base");
            ac.sendGoal(goal);
            ac.waitForResult();

            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO_STREAM("Goal " << (i + 1) << "/" << current_room_disinfection_pose.size() << " reached");
            }
            else
            {
                ROS_WARN_STREAM("Failed to reach goal " << (i + 1) << "/" << current_room_disinfection_pose.size());
                // Handle failure here if needed
            }
        }

        ROS_INFO_STREAM("Disinfection completed");
        return NodeStatus::SUCCESS;
    }

    // Cleaning up disinfection data if halt was issued
    void FollowDisinfectionPath::cleanup(bool halted)
    {
        if (halted)
        {
            m_iterations = 1;
        }
    }

    // Mission halt implementation
    void FollowDisinfectionPath::halt()
    {
        ROS_INFO_STREAM("Disinfection halted");
        cleanup(true);
        // Do not forget to call this at the end.
        CoroActionNode::halt();
    }

} // namespace done
