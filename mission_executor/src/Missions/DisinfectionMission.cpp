#include <ros/ros.h>
#include <ros/package.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "std_msgs/String.h"
#include "mission_msgs/ActionConfirmation.h"
#include "mission_msgs/Room.h"

#include "Missions/DisinfectionMission.h"
#include <boost/uuid/uuid.hpp>            // uuid class
#include <boost/uuid/uuid_generators.hpp> // generators
#include <boost/uuid/uuid_io.hpp>
#include <thread>
#include <vector>

namespace done
{

    using namespace BT;

    DisinfectionMission::DisinfectionMission()
        : m_isRoomCorrect(false), m_roomId(0)
    {
        set_type(Mission::DISINFECTROOM);
    }

    // Get the nama of task enum as string
    std::string DisinfectionMission::taskToString(Task task)
    {
        switch (task)
        {
        case Task::NAVIGATION:
            return "Navigation";
        case Task::LAMP_HEATUP:
            return "Lamp heat up";
        case Task::DISINFECTING_ROOM:
            return "Disinfecting room";
        default:
            return "Unknown";
        }
    }

    // Read mission specific data from message
    void DisinfectionMission::readSpecificData()
    {
        set_type(Mission::DISINFECTROOM);

        try
        {
            nlohmann::json json = json.parse(getMissionArguments());

            m_roomId = 0;

            setRoomData(json);
        }
        catch (const std::exception &e)
        {
            ROS_INFO_STREAM(e.what());
        }
    }

    void DisinfectionMission::setRoomData(nlohmann::json data)
    {
        // Define a room and pose used in the Mission_executor
        Room room;
        geometry_msgs::PoseStamped pose;
        geometry_msgs::PoseStamped waiting_pose;
        pose.header.frame_id = "map";
        // {'mission_id': 100, 'mission_type': 5, 'priority': 50, 'mission_args': '{"room_id": 123, "room_coordinates": {"position": {"x": 4.9685564041137695, "y": 3.5239384174346924, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": -0.9499797607165545, "w": 0.3123114698965086}}}'}
        // Get the values from the JSON object in the mission_args key
        pose.pose.position.x = data["room_coordinates"]["position"]["x"];
        pose.pose.position.y = data["room_coordinates"]["position"]["y"];
        pose.pose.position.z = data["room_coordinates"]["position"]["z"];
        pose.pose.orientation.x = data["room_coordinates"]["orientation"]["x"];
        pose.pose.orientation.y = data["room_coordinates"]["orientation"]["y"];
        pose.pose.orientation.z = data["room_coordinates"]["orientation"]["z"];
        pose.pose.orientation.w = data["room_coordinates"]["orientation"]["w"];

        waiting_pose.header.frame_id = "map";
        waiting_pose.pose.position.x = data["wait_coordinates"]["position"]["x"];
        waiting_pose.pose.position.y = data["wait_coordinates"]["position"]["y"];
        waiting_pose.pose.position.z = data["wait_coordinates"]["position"]["z"];
        waiting_pose.pose.orientation.x = data["wait_coordinates"]["orientation"]["x"];
        waiting_pose.pose.orientation.x = data["wait_coordinates"]["orientation"]["y"];
        waiting_pose.pose.orientation.z = data["wait_coordinates"]["orientation"]["z"];
        waiting_pose.pose.orientation.w = data["wait_coordinates"]["orientation"]["w"];
        // ROS_INFO_STREAM("waiting_pose.pose.position.x" << waiting_pose.pose.position.x);

        room.roomPose = pose;
        room.roomwaitPose = waiting_pose;

        room.roomName = data["room_id"];
        ros::param::set("/roomid", room.roomName);

        std::string json_string = data.dump();
        ros::param::set("/json_data", json_string);

        room.dose = data["dose"];
        std::string room_dose_string;
        room_dose_string = std::to_string(room.dose);
        ros::param::set("/roomdose", room_dose_string);

        room.jobid = data["job_id"];
        ros::param::set("/jobid", room.jobid);

        room.part = data["part"];
        ros::param::set("/part", room.part);

        // Extract disinfection_coordinates
        std::vector<geometry_msgs::PoseStamped> disinfection_poses;
        for (auto &coordinate : data["disinfection_coordinates"])
        {
            geometry_msgs::PoseStamped disinfection_pose;
            disinfection_pose.header.frame_id = "map";
            disinfection_pose.pose.position.x = coordinate["position"]["x"];
            disinfection_pose.pose.position.y = coordinate["position"]["y"];
            disinfection_pose.pose.position.z = coordinate["position"]["z"];
            disinfection_pose.pose.orientation.x = coordinate["orientation"]["x"];
            disinfection_pose.pose.orientation.y = coordinate["orientation"]["y"];
            disinfection_pose.pose.orientation.z = coordinate["orientation"]["z"];
            disinfection_pose.pose.orientation.w = coordinate["orientation"]["w"];
            disinfection_poses.push_back(disinfection_pose);
            ROS_INFO_STREAM("disinfection_pose = " << disinfection_pose);
        }
        int Number_of_disinfection_points = disinfection_poses.size();
        ROS_INFO_STREAM("Number of disinfection points: " << disinfection_poses.size());
        ros::param::set("/Number_of_disinfection_points", Number_of_disinfection_points);

        room.disinfectionPoses = disinfection_poses; // Assuming the Room class has this member

        ROS_INFO_STREAM("room_pose = " << pose);
        ROS_INFO_STREAM("room_wait_pose = " << waiting_pose);

        // set the global variable current_room the the room specified in the mission
        current_room = room;
    }

    void DisinfectionMission::init()
    {

        // variable left from when room_id was parameter for a yaml file which contained the room coordinates
        m_isRoomCorrect = false;
    }

    bool DisinfectionMission::getUserConfirmation(ros::NodeHandle &nh, const std::string &job_id, const std::string &robot_id, const std::string &action_id, const std::string &action_type)
    {
        // Create a service client
        ros::ServiceClient client = nh.serviceClient<mission_msgs::ActionConfirmation>("action_confirmation_service");

        // Prepare the service request
        mission_msgs::ActionConfirmation srv;
        srv.request.job_id = job_id;
        srv.request.robot_id = robot_id;
        srv.request.action_id = action_id;
        srv.request.action_type = action_type;

        // Call the service
        if (client.call(srv))
        {
            return srv.response.confirmation;
        }
        else
        {
            ROS_ERROR("Failed to call the ActionConfirmation service");
            return false;
        }
    }

    /////////// Behavior tree nodes  //////////////
    void DisinfectionMission::updateGoalStatus()
    {
        while (ros::ok)
        {
            actionlib_msgs::GoalStatusArray::ConstPtr statusArray = ros::topic::waitForMessage<actionlib_msgs::GoalStatusArray>("/move_base/status", ros::Duration(0.1));
            if (statusArray != nullptr)
            {
                for (const auto &status : statusArray->status_list)
                {
                    if (status.goal_id.id == navigation.getGoalId() && status.status == actionlib_msgs::GoalStatus::SUCCEEDED)
                    {
                        m_goalReached = true;
                        break;
                    }
                    else if (status.goal_id.id == navigation.getGoalId())
                    {
                        m_goalReached = false;
                        break;
                    }
                }
            }
        }
    }

    NodeStatus DisinfectionMission::navigate()
    {
        NodeStatus navStatus = navigation.get_status();
        switch (navStatus)
        {
        case NodeStatus::SUCCESS:
            navigation.setIdle();
            m_isRoomCorrect = true;
            break;

        case NodeStatus::FAILURE:
            // if navigation fails - mission fails
            this->set_status(MissionStatus::FAILED);
            navigation.setIdle();
            m_isRoomCorrect = true;
            break;

        case NodeStatus::IDLE:
        {
            set_activeTask(Task::NAVIGATION);
            ROS_INFO_STREAM("Starting navigation to disinfection waiting pose.");
            // create unique goalID
            std::string wait_pose_goal_id = "disinfection wait_pose" + std::to_string(ros::Time::now().toSec());

            std::string room_pose_id = "disinfection room_pose" + std::to_string(ros::Time::now().toSec());

            std::string robot_id = "ff64b388-c4de-40b7-8089-4520d20e86e5"; // Replace with the appropriate robot ID
            boost::uuids::uuid temp_uuid = boost::uuids::random_generator()();
            const std::string uuid_string = boost::uuids::to_string(temp_uuid);
            std::string action_id = uuid_string; // Replace with the appropriate action ID
            std::string action_type = "wait_to_enter";

            // Create a service client object
            ros::ServiceClient client = m_pNh.serviceClient<mission_msgs::ActionConfirmation>("action_confirmation_service");

            // Create a request object and fill in the necessary fields
            mission_msgs::ActionConfirmation srv;
            // std::string job_id;
            // ros::param::get("/jobid", job_id);
            srv.request.job_id = current_room.jobid;
            srv.request.robot_id = robot_id;
            srv.request.action_id = action_id;
            srv.request.action_type = action_type;
            ROS_INFO_STREAM("request = " << srv.request);

            // ros::param::set("")

            // Call the service and process the response
            navigation.setGoalId(wait_pose_goal_id);
            m_pNh.setParam("Goal_id", room_pose_id);
            navigation.goTo(current_room.roomwaitPose);
            navStatus = BT::NodeStatus::FAILURE;
            bool service_call_successful = false;
            while (!service_call_successful)
            {
                // Call the service and process the response
                // updateGoalStatus();
                m_pNh.getParam("goal_reached", m_goalReached);
                ROS_INFO_STREAM("m_goalReached = " << m_goalReached);
                if (m_goalReached)
                {
                    m_goalReached = false;
                    if (client.call(srv))
                    {
                        service_call_successful = true;

                        bool user_confirmation = srv.response.confirmation;

                        ROS_INFO_STREAM("Goal Reached waiting for confirmation");
                        if (user_confirmation)
                        {
                            // create unique goalID
                            navigation.setGoalId(room_pose_id);
                            m_pNh.setParam("Goal_id", room_pose_id);

                            navigation.goTo(current_room.roomPose);
                            navStatus = BT::NodeStatus::FAILURE;
                            ROS_ERROR("Service called is confirmation true");
                        }
                        else
                        {
                            ROS_INFO_STREAM("User denied the action. Not entering the room.");
                            navStatus = BT::NodeStatus::FAILURE;
                        }
                    }
                    else
                    {

                        ROS_WARN("Failed to call action_confirmation_service. Retrying...");
                        ros::Duration(1.0).sleep(); // Sleep for a while before retrying
                    }
                }
                else
                {
                    ROS_INFO_STREAM("goal not reached yet");
                    // Handle the case when the user denies the action
                }
            }

            break;
        }

        default:
            break;
        }
        return navStatus;
    }

    // Behavior tree mock action node
    
    BT::NodeStatus DisinfectionMission::getDisinfectionPath(TreeNode &self)
    {
        ROS_INFO_STREAM("Getting disinfection path pssssssssss" << current_room.roomwaitPose);

        // Check if the disinfectionPoses vector is not empty.
        if (!current_room.disinfectionPoses.empty())
        {
            // If it is not empty, the function returns success.
            ROS_INFO_STREAM("Disinfection path retrieval SUCCESSFUL: Disinfection poses available.");
            self.setOutput("current_room_disinfection_pose",current_room.disinfectionPoses);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            // If it is empty, the function returns failure.
            ROS_ERROR("Disinfection path retrieval FAILED: No disinfection poses available.");
            return BT::NodeStatus::FAILURE;
        }
    }

    // Behavior tree mock action node
    NodeStatus DisinfectionMission::warmUpLamps()
    {
        ROS_INFO_STREAM("Warming up lamps");
        set_activeTask(Task::DISINFECTING_ROOM);
        return BT::NodeStatus::SUCCESS;
    }

    // Behavior tree mock action node
    NodeStatus DisinfectionMission::turnOffLamps()
    {
        ROS_INFO_STREAM("Turning lamps off");
        return BT::NodeStatus::SUCCESS;
    }

    // Behavior tree mock action node
    NodeStatus DisinfectionMission::isClearToDisinfect()
    {

        ROS_INFO_STREAM("Heyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy");
        return BT::NodeStatus::SUCCESS;
    }

    // Behavior tree mock condition node
    NodeStatus DisinfectionMission::isRoomPositionCorrect()
    {
        if (m_isRoomCorrect)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    // Behavior tree node for checking disinfection flag
    NodeStatus DisinfectionMission::isDisinfectionFinished(TreeNode &self)
    {
        Optional<bool> msg = self.getInput<bool>("isDisinfectionFinished");
        if (!msg)
        {
            return NodeStatus::FAILURE;
        }
        if (msg.value())
        {
            self.setOutput<bool>("isDisinfectionFinished", false);
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }

    // Set the active task
    void DisinfectionMission::set_activeTask(Task task)
    {
        m_activeTask = task;
    }

    // Get std::string representation of currently active mission name
    std::string DisinfectionMission::get_activeTask()
    {
        return taskToString(m_activeTask);
    }

} // namespace done
