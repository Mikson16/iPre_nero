"""
nero_sim_ctrl.launch.py
=======================
Igual que nero_can_ctrl.launch.py, pero en vez del driver real (agx_arm_ctrl
por CAN) levanta el simulador MuJoCo + RViz (bringup_nero) y los nodos de
nero_ctrl (main_node, can_module).

  ros2 launch agx_arm_sim_description nero_sim_ctrl.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("agx_arm_sim_description"),
                "launch",
                "bringup_nero.launch.py",
            )
        ),
    )

    return LaunchDescription([
        bringup,
        Node(package="nero_ctrl", executable="main_node", name="main_node", output="screen"),
        Node(package="nero_ctrl", executable="can_module", name="can_module", output="screen"),
    ])