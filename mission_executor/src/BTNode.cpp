#include <ros/package.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include "BTNodes/MissionInterface.h"
#include "BTNodes/SafetyInterface.h"
#include "BTNodes/ErrorMessage.h"
#include "BTNodes/FollowDisinfectionPath.h"
#include "BTNode.h"

namespace done
{

  using namespace BT;

  BTNode::BTNode()
  {
  }

  bool BTNode::init()
  {
    BehaviorTreeFactory factory;

    // Behavior tree mock node for error handling
    factory.registerNodeType<ErrorMessage>("ErrorMessage");

    // Mission nodes
    std::shared_ptr<MissionInterface> mission = std::make_shared<MissionInterface>();
    PortsList out_port = {OutputPort<std::string>("activeMission")};
    factory.registerSimpleCondition("IsMissionActive", std::bind(&MissionInterface::isMissionActive, mission, std::placeholders::_1), out_port);

    // Safety nodes
    SafetyInterface safety;
    PortsList in_mission_port = {InputPort<std::string>("activeMission")};
    std::shared_ptr<EmergencyStopMission> esMission = mission->m_missionContainer.getData<std::shared_ptr<EmergencyStopMission>>("EmergencyStopOff");
    factory.registerSimpleCondition("IsPersonDetected", std::bind(&SafetyInterface::isPersonDetected, &safety));
    factory.registerSimpleAction("CancelAll", std::bind(&EmergencyStopMission::cancelAll, esMission));

    // Battery charging mission nodes
    std::shared_ptr<BatteryChargingMission> bcMission = mission->m_missionContainer.getData<std::shared_ptr<BatteryChargingMission>>("Charging");
    factory.registerSimpleCondition("MoveToCharge", std::bind(&BatteryChargingMission::navigate, bcMission));
    factory.registerSimpleAction("ChargeBattery", std::bind(&BatteryChargingMission::charge, bcMission));
    factory.registerSimpleCondition("IsBatteryLow", std::bind(&BatteryChargingMission::isBatteryLow, bcMission));
    factory.registerSimpleCondition("IsBatteryFull", std::bind(&BatteryChargingMission::isBatteryFull, bcMission));

    // Disinfection mission nodes
    std::shared_ptr<DisinfectionMission> disMission = mission->m_missionContainer.getData<std::shared_ptr<DisinfectionMission>>("Disinfection");
    PortsList in_out_port_disinfection = {InputPort<std::string>("isDisinfectionFinished"), OutputPort<std::string>("isDisinfectionFinished")};
    PortsList disinfection_poses_input_port = {InputPort<std::vector<geometry_msgs::PoseStamped>>("current_room_disinfection_pose")};
    PortsList disinfection_poses_output_port = {OutputPort<std::vector<geometry_msgs::PoseStamped>>("current_room_disinfection_pose")};
    factory.registerSimpleCondition("IsDisinfectionFinished", std::bind(&DisinfectionMission::isDisinfectionFinished, disMission, std::placeholders::_1), in_out_port_disinfection);
    factory.registerSimpleCondition("MoveToDisinfect", std::bind(&DisinfectionMission::navigate, disMission));
    factory.registerSimpleCondition("GetDisinfectionPath", std::bind(&DisinfectionMission::getDisinfectionPath, disMission, std::placeholders::_1), disinfection_poses_output_port);
    factory.registerSimpleCondition("WarmUpLamps", std::bind(&DisinfectionMission::warmUpLamps, disMission));
    factory.registerSimpleCondition("TurnOffLamps", std::bind(&DisinfectionMission::turnOffLamps, disMission));
    factory.registerSimpleCondition("IsClearToDisinfect", std::bind(&DisinfectionMission::isClearToDisinfect, disMission));
    factory.registerSimpleCondition("IsRoomPositionCorrect", std::bind(&DisinfectionMission::isRoomPositionCorrect, disMission));
    factory.registerNodeType<FollowDisinfectionPath>("FollowDisinfectionPath");

    // Load tree
    std::string path = ros::package::getPath("mission_executor") + "/behavior/MainTree.xml";
    try
    {
      // Creating behavior tree
      auto tree = factory.createTreeFromFile(path);

      NodeStatus status = NodeStatus::RUNNING;
      ros::Rate loopRate(1);

      while (ros::ok())
      {
        ros::spinOnce();
        status = tree.tickRoot();

        // If tree return success - some mission ended and it need finishing up
        if (status == NodeStatus::SUCCESS)
        {
          mission->setMissionStatus(MissionBase::MissionStatus::SUCCESSFUL);
        }

        loopRate.sleep();
      }
    }
    catch (const std::exception &e)
    {
      ROS_INFO_STREAM("Unable to load behavior tree xml file: " << e.what());
      return false;
    }

    return true;
  }

} // namespace done
