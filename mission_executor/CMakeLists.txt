cmake_minimum_required(VERSION 3.0.2)
project(mission_executor)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  geometry_msgs
  move_base_msgs
  behaviortree_cpp_v3
  mission_msgs
  actionlib
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES mission_receiver
  CATKIN_DEPENDS message_generation
   actionlib_msgs 
   geometry_msgs 
   roscpp
   rospy 
   sensor_msgs 
   std_msgs 
   mission_msgs 
   message_runtime
#  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}

  # ROS node
  src/BTNode.cpp
  src/BTMain.cpp
  

  # Behavior tree nodes
  src/BTNodes/ErrorMessage.cpp
  src/BTNodes/FollowDisinfectionPath.cpp
  src/BTNodes/MissionInterface.cpp
  src/BTNodes/NavigationInterface.cpp
  

  # Missions
  src/Missions/BatteryChargingMission.cpp
  src/Missions/DisinfectionMission.cpp
  src/Missions/EmergencyStopMission.cpp
  src/Missions/MissionBase.cpp
  src/Missions/MissionContainer.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(test_action_confirmation_client src/test_action_confirmation_client.cpp)
target_link_libraries(test_action_confirmation_client ${catkin_LIBRARIES})
add_dependencies(test_action_confirmation_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(navigationfeedback src/NavigationFeedback.cpp)
target_link_libraries(navigationfeedback ${catkin_LIBRARIES})


install(
  FILES
    launch/mission_executor.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
