#include <ros/ros.h>
#include <actionlib_msgs/GoalStatusArray.h>

class GoalStatusSubscriber
{
public:
    GoalStatusSubscriber()
    {
        ros::NodeHandle nh;
        goal_status_subscriber_ = nh.subscribe("/move_base/status", 10, &GoalStatusSubscriber::goalStatusCallback, this);
    }

private:
    void goalStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
    {
        if (msg->status_list.empty())
        {
            // No goals, set 'goal_reached' param to true
            ros::param::set("goal_reached", true);
        }
        else
        {
            // The last status in the list is the current one
            auto status = msg->status_list.back().status;
            if (status == actionlib_msgs::GoalStatus::SUCCEEDED)
            {
                ros::param::set("goal_reached", true);
            }
            else
            {
                ros::param::set("goal_reached", false);
            }
        }
    }

    ros::Subscriber goal_status_subscriber_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goal_status_node");
    GoalStatusSubscriber goalStatusSubscriber;

    ros::spin();

    return 0;
}
