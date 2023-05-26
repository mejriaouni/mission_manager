# mission_executor

## Launch package

Launch node (with magni_world and magni_nav running):

```bash
roslaunch mission_executor mission_executor.launch
```

Node is waiting for missions.

Missions can be commanded by /mission_executor/mission_action_server/goal topic as:

```bash
rostopic pub -1 /mission_executor/mission_action_server/goal mission_msgs/DoneMissionActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  mission:
    header:
      seq: 0
      stamp: {secs: 0, nsecs: 0}
      frame_id: ''
    mission_id: 0
    mission_type: 1
    priority: 0
    mission_seq: 0
    mission_args: '{\"room_id\": 0 }'"
```

Additional data (room id etc.) is provided in mission_arguments as json string.
Here data is the number of room to disinfect.
Rooms and their respective disinfection start poses are described in config folder in file rooms.yaml

### Available missions

mission_type:

- 1 - emergency stop on
- 2 - emergency stop off
- 3 - mission pause
- 4 - mission resume
- 5 - disinfect
- 6 - battery charge

#### Emergency stop mission

Emergency stop switches the flag on and off. Both will cancel current mission. Emergency stop on will not allow other missions to be run until emergency stop off is executed.

#### Mission pause/resume

The way mission pausing works is disinfection and battery charge mission save their mission information before execution.
If at any point of execution 'mission pause' is called those missions are stopped, and if 'mission resume' is called the saved mission information is used to rerun previous mission.

#### Disinfect mission

At this point disinfection mission navigates to the correct room and mocks safety checks, lamp heating, disinfection process.

#### Battery charge

Anytime during node execution battery charge can be set:

```bash
rostopic pub /charge_level std_msgs/Int16 "data: 4"
```

Behavior tree will react only to battery being bellow some level (if it is in disinfection mission) defined in config/charging.yaml.
At this point robot will navigate to the charging station and "charge". The previously run mission will have to be rerun.

### Action server

During the lifetime of the mission_executor node status of the missions can be tracked on topics:

```bash
rostopic echo /mission_executor/mission_action_server/feedback
rostopic echo /mission_executor/mission_action_server/result
```

## Requirements

```bash
sudo apt-get install -y nlohmann-json3-dev
sudo apt install ros-$ROS_DISTRO-behaviortree-cpp-v3
```

## Setting up Groot

Groot is used to create beahavior tree xml file. It is optional but helpful.

We've used the [this link](https://medium.com/teamarimac/groot-with-ros-a8f7855f8e35) to setup Groot.

ZeroMQ here is optional if you do not wish to follow active tree nodes in Groot during node execution.

In Groot environment you construct your tree using nodes that are ready to use (fallback nodes, sequence nodes, etc.) and action nodes you create yourself.
Some basic information on the nodes can be found [here](https://www.behaviortree.dev/).
