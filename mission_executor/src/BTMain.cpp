#include "BTNode.h"

using namespace done;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_executor");

    BTNode btNode;
    if (!btNode.init())
    {
        ROS_ERROR("pffffffffffffffff");
        return 1;
    }
    ros::spin();
    return 0;
}
