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

## Nero joint calibration

The calibration node requires Nero firmware V111 or newer and an active CAN
interface. Build the package after changing the calibration code:

```
cd ~/Projects/iPre_nero
colcon build --packages-select nero_ctrl
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Start the calibration node in one terminal. The selected joint or joints are
disabled when the node starts:

```
ros2 launch nero_ctrl nero_calibration.launch.py \
	channel:=can0 \
	bitrate:=1000000 \
	firmware:=v111 \
	joint_index:=1
```

Move the selected joint to its mechanical zero position. In a second terminal,
source the workspace and call the service:

```
source /opt/ros/jazzy/setup.bash
source ~/Projects/iPre_nero/install/setup.bash

ros2 service call /calibrate_joint std_srvs/srv/Trigger "{}"
```

The service has no request fields. It sets the current position as zero and
then attempts to enable the selected joint again.

Use `joint_index:=1` through `joint_index:=7` to calibrate one joint. Use
`joint_index:=255` to calibrate all seven joints; place every joint at its
mechanical zero position before calling the service:

```
ros2 launch nero_ctrl nero_calibration.launch.py \
	channel:=can0 \
	bitrate:=1000000 \
	firmware:=v111 \
	joint_index:=255
```

The expected successful response is similar to:

```
success: true
message: Joint 1 calibrated and enabled.
```

If enabling fails, do not assume that calibration was accepted. In firmware
V111 the calibration command does not provide an acknowledgement through the
SDK. Check CAN communication with:

```
candump -tz can0
```

Keep the calibration node running while calling the service, and do not run
another node that uses the same CAN interface at the same time.

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

## To send obj point (with gripper)
```
ros2 topic pub --once /input_obj_position std_msgs/msg/Float64MultiArray "{data: [-0.40, 0.0, 0.3, -95.0, 74.0, -179.0, 0.0]}"

```
[x, y, z, Rx, Ry, Rz, gripper_position]

## Launch nero_can_ctrl
```
bash ~/Projects/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh
source /opt/ros/jazzy/setup.bash
source ~/Projects/agx_arm_ws/install/setup.bash
source ~/Projects/iPre_nero/install/setup.bash

ros2 launch nero_ctrl nero_can_ctrl.launch.py
ros2 launch nero_ctrl nero_can_ctrl_ik.launch.py
```
