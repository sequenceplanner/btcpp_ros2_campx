# Behavior Tree CPP for CampX

1. Create a Behavior Tree with Groot 2 and save the .xml tree in:
```
btcpp_ros2_execution/behavior_trees
```

NOTE: The input/output ports have to be defined in:

```
btcpp_ros2_execution/src/execute_action.hpp
```
for example:
```
basic.insert(InputPort<std::string>("action_name", "", "Action server name"));
basic.insert(InputPort<unsigned>("msec"));
basic.insert(InputPort<std::string>("robot_position"));
```
2. Run the Rita suite:
```
ros2 launch rita_bringup bringup_sim.launch.py
```

<!-- 3. Run the ur_script_driver:
```
cd into ur_script_driver

find your ip_address

cargo run -- --ros-args -p override_host_address:=ip_address



``` -->

3. Run the Tree Execution Server
``` bash
ros2 launch btcpp_ros2_execution bt_executor.launch.xml
```
To call the Action Server from the command line:
``` bash
ros2 action send_goal /behavior_server btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: MoveRobotExample}"
```