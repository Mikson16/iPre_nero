# How to use the nero_ctrl package


## Using can communication

To activate can communication
```
"from workspace directory"
cd agx_arm_ws/src/agx_arm_ros/scripts 
bash can_activate.sh
```
Then

```
source /opt/ros/jazzy/setup.bash "from workspace directory"
source install/setup.bash "always in any terminal before running any ros2 command"
```

## Enable and disable the robot
To enable the arm, run the following command in a terminal:

```
ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"
```
To disable the arm, run the following command in a terminal:

```
ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: false}"
```

## To activate agx_arm_ctrl_single_node.py
```
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py can_port:=can0 arm_type:=nero effector_type:=agx_gripper tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]' control_enabled:=true
```
Source: https://github.com/agilexrobotics/agx_arm_ros/blob/ros2/README_EN.md

## To move the gripper manually

```
ros2 topic pub --once /control/move_j sensor_msgs/msg/JointState \
"{name: [gripper], position: [0.05], effort: [1.0]}"
```
Open: position: [0.10]
Half open: position: [0.05]
Close: position: [0.0]
Width range: 0.0 to 0.1 meters
Force range: 0.5 to 3.0 N

# Launch nero_can_ctrl
```
bash ~/Projects/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh
source /opt/ros/jazzy/setup.bash
source ~/Projects/agx_arm_ws/install/setup.bash
source ~/Projects/iPre_nero/install/setup.bash

ros2 launch nero_ctrl nero_can_ctrl.launch.py
ros2 launch nero_ctrl nero_can_ctrl_ik.launch.py
```
