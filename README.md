# iPre_nero


## Quick Start

### 1. Install Python SDK

```bash
git clone https://github.com/agilexrobotics/pyAgxArm.git
cd pyAgxArm
```

Choose the installation command based on your ROS version:

**Jazzy** installation command:

```bash
pip3 install . --break-system-packages
```

**Humble** installation command:

```bash
pip3 install .
```

### 2. Install Dependencies

Run the script to install all dependencies at once:

```bash
cd ~/iPre_nero/src/agx_arm_ros/scripts/
chmod +x agx_arm_install_deps.sh
bash ./agx_arm_install_deps.sh
```

Or install manually by executing the following commands in order:

1. Python dependencies

    Choose the installation command based on your ROS version:

    **Jazzy** installation command:

    ```bash
    pip3 install python-can scipy numpy --break-system-packages
    ```

    **Humble** installation command:

    ```bash
    pip3 install python-can scipy numpy
    ```

    > If using `effector_type:=revo2_pro` or `revo2_touch`, you also need [bc-stark-sdk](https://pypi.org/project/bc-stark-sdk/).

2. CAN tools

    ```bash
    sudo apt update && sudo apt install can-utils ethtool
    ```

3. ROS2 dependencies

    ```bash
    sudo apt install -y \
        ros-$ROS_DISTRO-ros2-control \
        ros-$ROS_DISTRO-ros2-controllers \
        ros-$ROS_DISTRO-controller-manager \
        ros-$ROS_DISTRO-topic-tools \
        ros-$ROS_DISTRO-joint-state-publisher-gui \
        ros-$ROS_DISTRO-robot-state-publisher \
        ros-$ROS_DISTRO-xacro \
        python3-colcon-common-extensions
    ```

4. MoveIt

    Before using MoveIt, you need to configure the related dependencies. For detailed steps, please refer to: [agx_arm_moveit](./src/agx_arm_moveit/README_EN.md)
    
    Or execute the following commands in order:

    ```bash
    sudo apt install ros-$ROS_DISTRO-moveit*
    ```

    ```bash
    sudo apt-get install -y \
        ros-$ROS_DISTRO-control* \
        ros-$ROS_DISTRO-joint-trajectory-controller \
        ros-$ROS_DISTRO-joint-state-* \
        ros-$ROS_DISTRO-gripper-controllers \
        ros-$ROS_DISTRO-trajectory-msgs
    ```

    If the system locale is not set to English, it must be set to English locale:

    ```bash
    echo "export LC_NUMERIC=en_US.UTF-8" >> ~/.bashrc
    source ~/.bashrc
    ```

### 3. Build and Source Workspace

Check if you are in a virtual environment. If so, it is recommended to exit the virtual environment first.

```bash
which pip3
```

Build and Source the workspace:

```bash
cd ~/iPre_nero
colcon build
source install/setup.bash
```

## Simulator
### 1. Source workspace
```bash
source /opt/ros/jazzy/setup.bash
source /opt/ros/humble/setup.bash
source ~/iPre_nero/install/setup.bash
```
### 2. Launch nero_can_ctrl.launch.py

```bash
ros2 launch nero_ctrl nero_can_ctrl.launch.py mode:=sim
```
Without rviz
```bash
ros2 launch nero_ctrl nero_can_ctrl.launch.py mode:=sim rviz:=false
```
