#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include "BTNodes/ErrorMessage.h"

namespace done
{

using namespace BT;

ErrorMessage::ErrorMessage(const std::string& name)
: BT::SyncActionNode(name, {})
{
}

// Behavior tree mock action node for error message
BT::NodeStatus ErrorMessage::tick()
{
    ROS_INFO_STREAM("System encountered an error");
    return BT::NodeStatus::FAILURE;
}


}// namespace done


