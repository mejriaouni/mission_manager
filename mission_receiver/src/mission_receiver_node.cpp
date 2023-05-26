#include "mission_receiver/mission_receiver.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mission_receiver");
    ros::NodeHandle pnh("~");
    ros::NodeHandle nh;

    std::string as_name;
    if(nh.getParam("action_server_name", as_name))
    {
        ROS_INFO("ACTION SERVER NAME PARSED: %s" , as_name.c_str());
    }
    else{
        ROS_INFO("Parsing action server name failed.");
        ros::shutdown();
    }
    mission_manager::MissionReceiver mission_receiver(pnh,nh,as_name);
    ros::spin();
}
