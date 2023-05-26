cmake_minimum_required(VERSION 3.0.2)
project(mission_receiver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  mission_msgs
  actionlib
  actionlib_msgs
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES mission_receiver
  CATKIN_DEPENDS actionlib_msgs
  geometry_msgs
  roscpp
  rospy 
  sensor_msgs
  std_msgs
  mission_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
  src/mission_receiver_node.cpp
  src/mission_receiver.cpp
)
add_dependencies(${PROJECT_NAME}_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# test code
add_executable(${PROJECT_NAME}_test
src/test_queue.cpp
)
add_dependencies(${PROJECT_NAME}_test 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_test
  ${catkin_LIBRARIES} 
)

# test for whole pipeline
add_executable(mission_manager_test
src/test_missionManager.cpp
)
add_dependencies(mission_manager_test 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mission_manager_test
  ${catkin_LIBRARIES} 
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(utest_node test/utest.test src/test/utest.cpp)
  add_library(mission_receiver_lib src/mission_receiver.cpp)
  target_link_libraries(utest_node ${catkin_LIBRARIES} mission_receiver_lib)
endif()
