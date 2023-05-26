#ifndef _NAVIGATIONINTERFACE_H_
#define _NAVIGATIONINTERFACE_H_

#include <actionlib_msgs/GoalStatusArray.h>
namespace done
{

class NavigationInterface
{

public:

    NavigationInterface();

    // Getting status of navigation
    BT::NodeStatus get_status();

    // Getting GoalId of navigation
    std::string getGoalId();
    // Setting navigation to ready mode
    void setIdle();

    // Cancel all move base goals
    void cancelAllGoals();

    // Setting goal ID for current navigation attempt
    void setGoalId(std::string goal_id);

    // Publish goal pose to move_base
    void goTo(const geometry_msgs::PoseStamped pose);

    // Initialize ROS subscribers and publishers
    void initROS();

    // Callback for move_base navigation status
    void feedbackCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

private:

    /// Node handle
    ros::NodeHandle m_pNh;

    // ActionLib subscriber
    ros::Subscriber m_feedbackSub;

    /// Goal publisher
    ros::Publisher m_goalPub;

    /// Cancel publisher
    ros::Publisher m_cancelPub;

    // Current navigation status
    BT::NodeStatus m_status;

    // Id for current goal
    std::string m_goalId;

};
}//namespace done

#endif
