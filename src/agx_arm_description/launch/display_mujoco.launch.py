"""
display_mujoco.launch.py
=========================
Lanza el simulador MuJoCo + puente ROS2 del brazo Nero (mujoco_viewer.py).

No depende de donde este instalada la carpeta mujoco/: busca la escena y el script
en este orden y usa el primero que exista (e imprime cual eligio).

Escena  (<scene> dentro de agilex_arm/agilex_nero/):
  1. argumento  model_dir:=/ruta/a/agilex_arm/agilex_nero
  2. variable de entorno NERO_MUJOCO_DIR
  3. share/<pkg>/mujoco/agilex_arm/agilex_nero  (agx_arm_description)
  4. buscando hacia arriba desde este archivo, en cada carpeta ancestro, la ruta
     mujoco/agilex_arm/agilex_nero o src/mujoco/agilex_arm/agilex_nero
     (funciona con --symlink-install y con instalacion normal, en cualquier equipo)

Script (mujoco_viewer.py):
  1. argumento  viewer_script:=/ruta/mujoco_viewer.py
  2. ../scripts/mujoco_viewer.py junto a este launch (con --symlink-install apunta a src)
  3. share/<pkg>/scripts/mujoco_viewer.py

Ejemplos:
  ros2 launch agx_arm_description display_mujoco.launch.py scene:=scene_nero_arm.xml
  ros2 launch agx_arm_description display_mujoco.launch.py model_dir:=/ruta/a/mujoco/agilex_arm/agilex_nero
"""

import os
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# agx_arm_sim_description ya no existe (migrado a agx_arm_description); no se busca para no
# cargar copias viejas que queden instaladas en otro workspace.
PACKAGES = ("agx_arm_description",)
MODEL_SUBDIR = os.path.join("mujoco", "agilex_arm", "agilex_nero")


def _share(pkg):
    try:
        return get_package_share_directory(pkg)
    except PackageNotFoundError:
        return None


def _workspace_model_dirs():
    """Carpetas candidatas del modelo subiendo desde este archivo (sin rutas fijas).

    Con --symlink-install, realpath(__file__) apunta a src/ y se encuentra src/mujoco/...;
    con instalacion normal sube hasta la raiz del workspace y prueba <raiz>/src/mujoco/...
    """
    this_file = globals().get("__file__")
    if not this_file:
        return []
    found = []
    d = Path(this_file).resolve().parent
    while True:
        found.append(str(d / MODEL_SUBDIR))
        found.append(str(d / "src" / MODEL_SUBDIR))
        if d.parent == d:
            break
        d = d.parent
    return found


def launch_setup(context, *args, **kwargs):
    scene = LaunchConfiguration("scene").perform(context)
    model_dir_arg = LaunchConfiguration("model_dir").perform(context)
    script_arg = LaunchConfiguration("viewer_script").perform(context)

    # ---- escena ----
    model_dirs = []
    if model_dir_arg:
        model_dirs.append(os.path.expanduser(model_dir_arg))
    if os.environ.get("NERO_MUJOCO_DIR"):
        model_dirs.append(os.path.expanduser(os.environ["NERO_MUJOCO_DIR"]))
    for pkg in PACKAGES:
        share = _share(pkg)
        if share:
            model_dirs.append(os.path.join(share, MODEL_SUBDIR))
    model_dirs.extend(_workspace_model_dirs())

    scene_file = next(
        (os.path.join(d, scene) for d in model_dirs if os.path.isfile(os.path.join(d, scene))),
        None,
    )
    if scene_file is None:
        raise FileNotFoundError(
            f"[display_mujoco] No se encontro la escena '{scene}'. Carpetas probadas:\n  "
            + "\n  ".join(model_dirs)
            + "\nUsa model_dir:=<carpeta> o define NERO_MUJOCO_DIR."
        )

    # ---- script del viewer/bridge ----
    scripts = []
    if script_arg:
        scripts.append(os.path.expanduser(script_arg))
    this_file = globals().get("__file__")
    if this_file:
        scripts.append(str(Path(this_file).resolve().parent.parent / "scripts" / "mujoco_viewer.py"))
    for pkg in PACKAGES:
        share = _share(pkg)
        if share:
            scripts.append(os.path.join(share, "scripts", "mujoco_viewer.py"))

    script_path = next((s for s in scripts if os.path.isfile(s)), None)
    if script_path is None:
        raise FileNotFoundError(
            "[display_mujoco] No se encontro mujoco_viewer.py. Rutas probadas:\n  "
            + "\n  ".join(scripts)
            + "\nUsa viewer_script:=<ruta>."
        )

    return [
        LogInfo(msg=f"[display_mujoco] escena: {scene_file}"),
        LogInfo(msg=f"[display_mujoco] script: {script_path}"),
        ExecuteProcess(cmd=["python3", script_path, scene_file], output="screen"),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "scene",
            default_value="scene.xml",
            choices=["scene.xml", "scene_nero_arm.xml", "nero.xml", "nero_arm.xml"],
            description="Archivo MuJoCo a cargar dentro de agilex_arm/agilex_nero/",
        ),
        DeclareLaunchArgument(
            "model_dir",
            default_value="",
            description="Carpeta agilex_arm/agilex_nero con los XML (opcional)",
        ),
        DeclareLaunchArgument(
            "viewer_script",
            default_value="",
            description="Ruta a mujoco_viewer.py (opcional)",
        ),
        OpaqueFunction(function=launch_setup),
    ])