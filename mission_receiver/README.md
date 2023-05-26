 # Mission Receiver
 
## Requirements

```bash
sudo apt-get install -y nlohmann-json3-dev
```

## Build 

```bash
cd path-to/catkin_ws/src
git clone https://gitlab.com/done-enterprises/service-robot/ros-ecosystem/modules/mission-manager.git
cd ..
catkin_make
source devel/setup.bash
```


## Testing

### Running mission queue test cases
```
catkin_make run_tests_mission_receiver
```

### Testing mission_receiver and mission_executor

Launch Gazebo and robot packages using
```
roslaunch mission_receiver test_envBringup.launch
```

Launch mission_receiver in a separate terminal
```
roslaunch mission_receiver mission_receiver.launch
```
Finally pass the test messages 
```
rosrun mission_receiver mission_manager_test
```
