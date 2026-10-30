"""
agx_arm_description — bringup_nero.launch.py
====================================================
Lanza en un solo comando:
  1. La fisica del brazo Nero en MuJoCo + puente ROS2 (mujoco_viewer.py), a traves de
     display_mujoco.launch.py (mismo directorio launch/).
  2. robot_state_publisher + RViz2 siguiendo `feedback/joint_states` (sin sliders).

NO incluye el display.launch.py de agx_arm_description: construye el URDF por su
cuenta a partir de los archivos del paquete, asi funciona igual con la version de
iPre_nero (urdf/agx_arm_description.urdf.xacro) que con la de agx_arm_ros
(agx_arm_urdf/nero/urdf/...). Los sliders (joint_state_publisher_gui) nunca se lanzan.

Ejemplos:
  ros2 launch agx_arm_description bringup_nero.launch.py
  ros2 launch agx_arm_description bringup_nero.launch.py scene:=scene_nero_arm.xml
  ros2 launch agx_arm_description bringup_nero.launch.py end_effector:=none
  ros2 launch agx_arm_description bringup_nero.launch.py rviz:=false   # sin RViz
"""

import os

import xacro
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_launch(name):
    """Busca <name> junto a este archivo y, si no, en launch/ de los paquetes conocidos."""
    candidates = []
    here = globals().get("__file__")
    if here:
        candidates.append(os.path.join(os.path.dirname(os.path.abspath(here)), name))
    for pkg in ("agx_arm_description",):
        try:
            candidates.append(os.path.join(get_package_share_directory(pkg), "launch", name))
        except PackageNotFoundError:
            pass
    for c in candidates:
        if os.path.isfile(c):
            return c
    raise FileNotFoundError(
        f"[bringup_nero] No se encontro {name}. Rutas probadas:\n  " + "\n  ".join(candidates)
    )


def _build_robot_description(desc_share, end_effector):
    """Devuelve el URDF (string) del Nero, probando los dos layouts conocidos."""
    # Layout iPre_nero: un unico xacro de entrada con mappings.
    legacy = os.path.join(desc_share, "urdf", "agx_arm_description.urdf.xacro")
    if os.path.exists(legacy):
        return xacro.process_file(
            legacy,
            mappings={
                "arm_type": "nero",
                "end_effector": end_effector,
                "revo2_side": "right",
                "with_camera_stand": "false",
                "with_camera": "false",
            },
        ).toxml()

    # Layout agx_arm_ros: un URDF/xacro por tipo de brazo.
    new_name = (
        "nero_with_gripper_description.xacro"
        if end_effector == "gripper"
        else "nero_description.urdf"
    )
    new = os.path.join(desc_share, "agx_arm_urdf", "nero", "urdf", new_name)
    if os.path.exists(new):
        return xacro.process_file(new).toxml()

    raise FileNotFoundError(
        "[bringup_nero] No se encontro el URDF del Nero en agx_arm_description.\n"
        f"  Probado: {legacy}\n  Probado: {new}\n"
        f"  Paquete usado: {desc_share}"
    )


def launch_setup(context, *args, **kwargs):
    end_effector = LaunchConfiguration("end_effector").perform(context)

    desc_share = get_package_share_directory("agx_arm_description")
    mujoco_launch = _find_launch("display_mujoco.launch.py")

    robot_description = _build_robot_description(desc_share, end_effector)

    rviz_config = None
    for name in ("default.rviz", "display.rviz"):
        candidate = os.path.join(desc_share, "rviz", name)
        if os.path.exists(candidate):
            rviz_config = candidate
            break

    mujoco_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mujoco_launch),
        launch_arguments={"scene": LaunchConfiguration("scene")}.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        remappings=[("joint_states", "feedback/joint_states")],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config] if rviz_config else [],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return [mujoco_sim, rsp, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "scene",
            default_value="scene.xml",
            choices=["scene.xml", "scene_nero_arm.xml", "nero.xml", "nero_arm.xml"],
            description="Escena MuJoCo a cargar (se pasa a display_mujoco.launch.py)",
        ),
        DeclareLaunchArgument(
            "end_effector",
            default_value="gripper",
            choices=["none", "gripper"],
            description="Efector final del URDF que ve RViz",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            choices=["true", "false"],
            description="Lanzar RViz2 (true/false). robot_state_publisher se lanza siempre",
        ),
        OpaqueFunction(function=launch_setup),
    ])