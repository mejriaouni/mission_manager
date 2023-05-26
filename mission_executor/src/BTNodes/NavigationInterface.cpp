#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include "BTNodes/NavigationInterface.h"

namespace done
{

using namespace BT;


NavigationInterface::NavigationInterface()
: m_pNh("~")
, m_status(NodeStatus::IDLE)
, m_goalId("not set")
{
    initROS();
}

// Getting status of navigation
NodeStatus NavigationInterface::get_status()
{
    return m_status;
}

// Getting GoalId of navigation
std::string NavigationInterface::getGoalId()
{
    return m_goalId;
}

void NavigationInterface::cancelAllGoals()
{
    actionlib_msgs::GoalID msg;
    m_cancelPub.publish(msg);
}

// Setting navigation to ready mode
void NavigationInterface::setIdle()
{
    cancelAllGoals();
    m_status = NodeStatus::IDLE;
}

// Setting goal ID for current navigation attempt
void NavigationInterface::setGoalId(std::string goal_id)
{
    m_goalId = goal_id;
}

// Publish goal pose to move_base
void NavigationInterface::goTo(const geometry_msgs::PoseStamped pose)
{
    move_base_msgs::MoveBaseActionGoal goalMsg;
    goalMsg.goal_id.stamp = ros::Time::now();
    goalMsg.goal_id.id = m_goalId;
    goalMsg.goal.target_pose = pose;
    m_goalPub.publish(goalMsg);

    m_status = NodeStatus::RUNNING;
}

// Initialize ROS subscribers and publishers
void NavigationInterface::initROS()
{
    m_cancelPub   = m_pNh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 100);
    m_goalPub     = m_pNh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 100);
    m_feedbackSub = m_pNh.subscribe("/move_base/status", 100, &NavigationInterface::feedbackCallback, this);
}

// Callback for move_base navigation status
void NavigationInterface::feedbackCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
    if (m_status==NodeStatus::RUNNING)
    {
        if (!msg->status_list.empty())
        {
            for (int z=0; z<msg->status_list.size(); z++)
            {
                if(msg->status_list[z].goal_id.id==m_goalId)
                {
                    switch (msg->status_list[z].status)
                    {
                        case 0:
                            ROS_INFO_STREAM("Preparing for navigation to goal '"<<m_goalId<<"'");
                            break;

                        case 1:
                            ROS_INFO_STREAM("Navigating to goal '"<<m_goalId<<"'");
                            break;

                        case 3:
                            ROS_INFO_STREAM("Navigation to goal '"<< m_goalId <<"' successful");
                            m_status = NodeStatus::SUCCESS;
                            break;

                        default:
                            ROS_INFO_STREAM("Navigation to '"<< m_goalId <<"' stopping: error occurred/goal canceled.");
                            m_status = NodeStatus::FAILURE;
                            break;
                    }
                    return;
                }
            }

        }
        else
        {
            ROS_INFO_STREAM("Navigation failed.");
            m_status = NodeStatus::FAILURE;
        }
    }
    return;
}

}//namespace done
