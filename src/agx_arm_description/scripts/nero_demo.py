import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


def skew_matrix(a):
    return np.array([[0, -a[2], a[1]],
                     [a[2], 0, -a[0]],
                     [-a[1], a[0], 0]])


def adjoint_matrix(R, p):
    adjoint = np.zeros((6, 6))
    adjoint[:3, :3] = R
    adjoint[3:, 3:] = R
    adjoint[:3, 3:] = -R @ skew_matrix(p)
    return adjoint


# Ruta del modelo relativa a la ubicacion de ESTE script, no al directorio desde
# donde se lo ejecute (funciona igual si corres "python3 nero_demo.py" parado en
# scripts/, o "python3 scripts/nero_demo.py" parado en la raiz del paquete).
SCRIPT_DIR = Path(__file__).resolve().parent
MODEL_PATH = SCRIPT_DIR.parent / 'mujoco' / 'agilex_arm' / 'agilex_nero' / 'scene_nero_arm.xml'

model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)

jacp = np.zeros((3, model.nv))
jacr = np.zeros((3, model.nv))
jac_all = np.zeros((6, model.nv))
jac_all_body = np.zeros((6, model.nv))

N_ARM_JOINTS = 7    # joint1 .. joint7
N_GRIPPER_JOINTS = 2  # gripper_joint1, gripper_joint2 (vienen DESPUES del brazo en qpos)
# El Nero tiene 2 DOF de pinza entre el brazo y la pelota (el Franka de tu ejemplo no los
# tenia), asi que el freejoint de la pelota arranca en el indice 7+2=9, no en 7.
BALL_QPOS_START = N_ARM_JOINTS + N_GRIPPER_JOINTS  # 9

init_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
q_ball = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0]  # pos (3) + quat (4) de la pelota, igual que q_esfera
init_q_vel = [0.0] * model.nv
goal = [0.3, 0.3, 0.3]
tol = 0.01
alpha = 0.005
body_id = model.body('link7').id
t = 0.0
data.qpos[:N_ARM_JOINTS] = init_q
data.qpos[BALL_QPOS_START:BALL_QPOS_START + 7] = q_ball
# data.qvel = init_q_vel
data.ctrl[:N_ARM_JOINTS] = init_q
mujoco.mj_forward(model, data)

# NOTA: la pelota de scene_nero_arm.xml tiene masa y colisiona (a diferencia de un
# marcador puramente visual). Teletransportarlo cada frame para que seale la meta
# puede verse inestable/con tirones al chocar contra el suelo o el brazo. Si solo
# lo quieres como marcador, lo mas limpio es convertirlo en un "mocap body" en el
# XML (sin fisica) y mover data.mocap_pos en vez de qpos -- puedo armarte eso si
# te sirve.

with mujoco.viewer.launch_passive(model, data) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() and time.time() - start < 30:
    step_start = time.time()

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mujoco.mj_step(model, data)

    mujoco.mj_jac(model, data, jacp, jacr, goal, body_id)
    mujoco.mj_jacBody(model, data, jac_all[:3], jac_all[3:], body_id)
    ori = data.xquat[body_id]
    print(ori)
    t += 0.005
    goal[2] = 0.3 + 0.2 * np.sin(t)
    goal2 = [0, 0.05 * np.sin(t), 0]

    # Jacobiana en el marco local del efector (usando la pose de link7, ya que el
    # Nero no tiene un site "attachment_site" como el Franka).
    jac_all_body = adjoint_matrix(data.xmat[body_id].reshape(3, 3), data.xpos[body_id]) @ jac_all

    pos = data.xpos[body_id]
    error = goal - pos

    jac_all_pinv = jac_all.T @ np.linalg.inv(jac_all @ jac_all.T)
    jac_all_body_pinv = jac_all_body.T @ np.linalg.inv(jac_all_body @ jac_all_body.T)

    jacp_pinv = jacp.T @ np.linalg.inv(jacp @ jacp.T)
    update = jacp_pinv @ error * alpha
    update2 = jac_all_pinv @ np.concatenate([error, np.zeros(3)]) * alpha
    update3 = jac_all_body_pinv @ np.concatenate([goal2, np.zeros(3)]) * alpha
    data.ctrl[:N_ARM_JOINTS] = data.ctrl[:N_ARM_JOINTS] + update2[:N_ARM_JOINTS]

    data.qpos[BALL_QPOS_START:BALL_QPOS_START + 7] = q_ball
    aux = goal.copy()
    aux[2] = aux[2] - 0.1
    data.qpos[BALL_QPOS_START:BALL_QPOS_START + 3] = aux
    print(jac_all_body)

    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
      viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)
      viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
      viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = model.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
      # t += time_until_next_step