#include <ros/ros.h>
#include "mission_msgs/ActionConfirmation.h"

bool handle_confirmation_request(mission_msgs::ActionConfirmation::Request &req,
                                 mission_msgs::ActionConfirmation::Response &res)
{
    // Set confirmation to true
    res.confirmation = true;
    ROS_INFO_STREAM("request = " << req);
    res.retry = false; // Set this according to your requirements
    ROS_INFO("Received request for job_id: %s, action_type: %s. Sending confirmation: %d", req.job_id.c_str(),
             req.action_type.c_str(),res.confirmation);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_confirmation_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("action_confirmation_service", handle_confirmation_request);
    ROS_INFO("Action confirmation server ready.");

    ros::spin();

    return 0;
}

