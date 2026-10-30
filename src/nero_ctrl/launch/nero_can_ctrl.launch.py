"""
nero_ctrl — nero_can_ctrl.launch.py
====================================
Mismo flujo (main_node + can_module) con el brazo real o con el simulador.

  mode:=real  (default)  agx_arm_ctrl por CAN -> brazo fisico
  mode:=sim              MuJoCo + RViz (bringup_nero.launch.py de agx_arm_sim_description)

Tanto agx_arm_ctrl como el bridge de MuJoCo escuchan `control/move_j` y publican
`feedback/joint_states`, asi que main_node y can_module no cambian entre modos.
NO uses los dos modos a la vez: ambos publicarian en feedback/joint_states.

Ejemplos:
  ros2 launch nero_ctrl nero_can_ctrl.launch.py
  ros2 launch nero_ctrl nero_can_ctrl.launch.py mode:=sim
  ros2 launch nero_ctrl nero_can_ctrl.launch.py mode:=sim scene:=scene_nero_arm.xml
  ros2 launch nero_ctrl nero_can_ctrl.launch.py mode:=sim rviz:=false   # sin RViz
  ros2 launch nero_ctrl nero_can_ctrl.launch.py mode:=real can_port:=can1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    can_port = LaunchConfiguration("can_port").perform(context)
    scene = LaunchConfiguration("scene").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context)
    start_delay = float(LaunchConfiguration("start_delay").perform(context))

    nodes = [
        Node(
            package="nero_ctrl",
            executable="main_node",
            name="main_node",
            output="screen",
        ),
        Node(
            package="nero_ctrl",
            executable="can_module",
            name="can_module",
            output="screen",
        ),
    ]

    if mode == "real":
        # Los paquetes del brazo real solo se buscan en este modo: para simular
        # no hace falta agx_arm_ctrl ni tener can0 activo.
        backend = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("agx_arm_ctrl"),
                    "launch",
                    "start_single_agx_arm.launch.py",
                )
            ),
            launch_arguments={
                "can_port": can_port,
                "arm_type": "nero",
                "effector_type": "agx_gripper",
                "tcp_offset": "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
                "control_enabled": "true",
            }.items(),
        )
        return [backend, *nodes]

    # mode == "sim"
    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("agx_arm_description"),
                "launch",
                "bringup_nero.launch.py",
            )
        ),
        launch_arguments={"scene": scene, "rviz": rviz}.items(),
    )
    # MuJoCo tarda en cargar; si can_module manda la pose inicial antes de que el
    # bridge se haya suscrito a control/move_j, ese mensaje se pierde. Se retrasan
    # main_node y can_module para que arranquen cuando el simulador ya escucha.
    return [backend, TimerAction(period=start_delay, actions=nodes)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="real",
            choices=["real", "sim"],
            description="real: brazo fisico por CAN | sim: simulador MuJoCo",
        ),
        DeclareLaunchArgument(
            "can_port",
            default_value="can0",
            description="Puerto CAN (solo mode:=real)",
        ),
        DeclareLaunchArgument(
            "scene",
            default_value="scene_nero_arm.xml",
            choices=["scene.xml", "scene_nero_arm.xml", "nero.xml", "nero_arm.xml"],
            description="Escena MuJoCo (solo mode:=sim)",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            choices=["true", "false"],
            description="Lanzar RViz2 (solo mode:=sim)",
        ),
        DeclareLaunchArgument(
            "start_delay",
            default_value="2.0",
            description="Segundos de espera antes de lanzar main_node/can_module (solo mode:=sim)",
        ),
        OpaqueFunction(function=launch_setup),
    ])