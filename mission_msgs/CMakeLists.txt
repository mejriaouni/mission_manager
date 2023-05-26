cmake_minimum_required(VERSION 3.0.2)
project(mission_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  actionlib_msgs
  roscpp
  rospy
  std_msgs
)

## Generate messages in the 'msg' folder
 add_message_files(
  FILES
  FeedbackMsg.msg
  MissionMsg.msg
  Room.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ActionConfirmation.srv
  MissionFeedback.srv
  RegisterMission.srv
  RegisterMissionOne.srv
)

add_action_files(
  FILES
  DoneMission.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  message_generation
  roscpp
  rospy
  std_msgs
  message_runtime
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
