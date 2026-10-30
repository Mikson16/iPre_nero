#!/usr/bin/env python3
import rclpy
import os
import sys
import time
import threading
import mujoco
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool, Float64MultiArray
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

"""This nodes handle the inverse kinematics calculation and send the reasult to
the controller node and then to the indermediary node"""

class InverseKinematicsNodeBodyJacobian(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        self.get_logger().info('Inverse Kinematics Node has been started.')

        #threads
        self.ik_condition = threading.Condition()
        self.pending_target = None
        self.stop_ik_thread = False

        self.ik_thread = threading.Thread(target=self.ik_worker, daemon=True)
        self.ik_thread.start()
        self.mujoco_lock = threading.Lock()
        # QoS profile 
        latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        model_path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        # atributes 
        self.timer_pr = 0.01 # 100Hz
        self.init_q = None
        self.init_vel = None
        self.current_q = None
        self.model_path = None
        self.robot_data = None
        self.model = None
        self.current_q = None
        self.body_ids = None
        self.ee_site_ids = None
        self.jacp = None 
        self.jacr = None
        self.ready_sent = False
        self.obj_point = None
        self.delayed_result_timer = None
        self.init_pos_get = False

        #publishers
        self.publisher_state = self.create_publisher(Bool, 'inverse_kinematics_ready', latch_qos)
        self.publisher_result = self.create_publisher(JointState, 'ik/ik_result', latch_qos)

        ik_qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)

        self.subscription = self.create_subscription(
            Bool,
            'start_running_nodes',
            self.start_running_nodes_callback,
            latch_qos,
        )
        
        self.state_subscription = self.create_subscription(
            JointState,
            'ik/joint_states',
            self.state_callback,
            ik_qos,
        )

        self.init_vel_subscription = self.create_subscription(Float64MultiArray, 'init_vel_robot', self.init_vel_callback, 10)

        self.obj_point_subscription = self.create_subscription(Float64MultiArray, '/obj_point', self.obj_point_callback, latch_qos)


        self.get_model_path_subscription = self.create_subscription(
            String,
            '/model_path',
            self.get_model_callback,
            model_path_qos,
        )
        self.init_subscription = self.create_subscription(Bool, 'start_running_nodes', self.start_running_nodes_callback, latch_qos)
    
    
    def ik_worker(self):
        while rclpy.ok():
            with self.ik_condition:
                self.ik_condition.wait_for(lambda: self.pending_target is not None or self.stop_ik_thread)
                if self.stop_ik_thread:
                    return
                target = self.pending_target.copy()
                self.pending_target = None
            with self.mujoco_lock:
                if self.model is not None and self.robot_data is not None:
                    self.calculate_inverse_kinematics(target)


    def obj_point_callback(self, msg):
        self.get_logger().info(f"Received object point: {msg.data}")
        with self.ik_condition:
            self.pending_target = np.array(msg.data, dtype=float)
            self.ik_condition.notify()

    def start_running_nodes_callback(self, msg):
        if msg.data and not self.ready_sent:
            self.get_logger().info('Starting Inverse Kinematics Node.')
            # # command for test
            # command = Float64MultiArray()
            # command.data = [0.0, 0.5, 0.0, -1.0, 0.0, 0.9, 0.0, 0.0] # initial pose
            # self.publisher_result.publish(command)

            self.publisher_state.publish(Bool(data=True))
            self.ready_sent = True

    # get initial pose when robot is ready
    def state_callback(self, msg):
        if not msg.position:
            return
        with self.mujoco_lock:
            self.current_q = np.array(msg.position, dtype=float)
            if not self.init_pos_get:
                self.get_logger().info('Robot is in initial state. Ready to receive target positions.')
                self.init_q = self.current_q.copy()
                self.init_pos_get = True
                self._try_assign_model_data()

            if self.robot_data is not None:
                qpos_count = min(len(self.current_q), self.model.nq)
                self.robot_data.qpos[:qpos_count] = self.current_q[:qpos_count]
                if msg.velocity:
                    qvel_count = min(len(msg.velocity), self.model.nv)
                    self.robot_data.qvel[:qvel_count] = np.array(msg.velocity[:qvel_count])
                mujoco.mj_forward(self.model, self.robot_data)

    # if needed and the rest of code is correct, you can use this to modify velocity, in the beginning of the system this can only be set by launch file
    def init_vel_callback(self, msg):
        if msg.data:
            self.get_logger().info('Robot initial velocity received.')
            # Here you can add logic to handle the robot's initial velocity if needed
            self.init_vel = np.array(msg.data)

    # Math functions
    def calculate_inverse_kinematics(self, target_position):
        self.get_logger().info(f"Calculating inverse kinematics for target position: {target_position}")
        if self.current_q is None:
            self.get_logger().error("Current joint positions are not available. Cannot perform inverse kinematics.")
            return
        if self.ee_site_ids is None or 'pinch_site' not in self.ee_site_ids:
            self.get_logger().error("End-effector site ID for 'pinch_site' is not available. Cannot perform inverse kinematics.")
            return
        if self.jacp is None or self.jacr is None:
            self.get_logger().error("Jacobian matrices are not initialized. Cannot perform inverse kinematics.")
            return

        # Get the target position and orientation
        target_pos = target_position[:3]
        target_R = self.rpy_to_rotation_matrix(target_position[3:6])
        arm_dof_count = min(len(self.current_q), 7, self.model.nv)

        q_copy = np.copy(self.robot_data.qpos[:])
        q_copy[:arm_dof_count] = self.current_q[:arm_dof_count]
        q_copy[7:] = np.nan_to_num(q_copy[7:], nan=0.0, posinf=0.0, neginf=0.0)
        ee_id = self.ee_site_ids['pinch_site']

        # using Newton Raphson method
        max_iter = 400
        tolerance = 1e-4
        alpha = 0.2 # 0.5
        converged = False

        max_step_cartesian = 0.05
        max_step_joint = 0.1

        arm_joint_ranges = self.model.jnt_range[:arm_dof_count]

        for i in range(max_iter):
            self.robot_data.qpos[:] = q_copy
            mujoco.mj_forward(self.model, self.robot_data)

            current_pos = self.robot_data.site_xpos[ee_id]
            current_R = self.robot_data.site_xmat[ee_id].reshape(3, 3)

            # Calculate error
            e_v = target_pos - current_pos
            R_err = target_R @ current_R.T
            angle = np.arccos(np.clip((np.trace(R_err) - 1) / 2.0, -1.0, 1.0)) #help of gemini :)
            if angle > 1e-6:
                axis = np.array([
                    R_err[2, 1] - R_err[1, 2],
                    R_err[0, 2] - R_err[2, 0],
                    R_err[1, 0] - R_err[0, 1]
                ])
                norm_axis = np.linalg.norm(axis)
                axis = axis / norm_axis if norm_axis > 1e-6 else np.array([0.0, 0.0, 1.0])
                e_omega = axis * angle
            else:
                e_omega = np.zeros(3)

            space_error = np.concatenate((e_v, e_omega)) # Twist

            # end condition
            if np.linalg.norm(space_error) < tolerance:
                converged = True
                self.get_logger().info(f"Inverse kinematics converged in {i} iterations.")
                break
            # limit error to avoid large jumps
            if np.linalg.norm(space_error) > max_step_cartesian:
                space_error = (space_error / np.linalg.norm(space_error)) * max_step_cartesian

            # Calculate Jacobian
            mujoco.mj_jacSite(self.model, self.robot_data, self.jacp, self.jacr, ee_id)
            J = np.vstack((self.jacp, self.jacr))[:, :arm_dof_count]

            #build adj matrix
            Ad_T = self.calculate_adj_matrix(current_R, current_pos)
            # inverse
            Ad_T_inv = np.linalg.inv(Ad_T)
            #jacobian of the body
            J_b = Ad_T_inv @ J

            body_error = Ad_T_inv @ space_error

            #NaN verification
            if not np.all(np.isfinite(J)):
                self.get_logger().error("Jacobian contains NaN or Inf values. IK result discarded.")
                return

            # Calculate the pseudo-inverse of the body Jacobian
            J_pseudo_inv = np.linalg.pinv(J_b)
            delta_q = J_pseudo_inv @ body_error

            # singularity protection
            norm_delta_q = np.linalg.norm(delta_q)
            if norm_delta_q > max_step_joint:
                delta_q = (delta_q / norm_delta_q) * max_step_joint

            q_copy[:arm_dof_count] += delta_q * alpha
            q_copy[:arm_dof_count] = np.clip(
                q_copy[:arm_dof_count],
                arm_joint_ranges[:, 0],
                arm_joint_ranges[:, 1],
            )
            # self.get_logger().info(f"jacp {self.jacp}, jacr {self.jacr}, J {J}, J_b {J_b}, ")
            if not np.all(np.isfinite(q_copy)):
                self.get_logger().error("IK produced a non-finite joint position; command was discarded.")
                return
            #ingresar a la iteracion
            self.robot_data.qpos[:] = q_copy
            mujoco.mj_forward(self.model, self.robot_data)
            final_position_error = np.linalg.norm(
                target_pos - self.robot_data.site_xpos[ee_id]
            )
            final_rotation = self.robot_data.site_xmat[ee_id].reshape(3, 3)
            final_rotation_error = target_R @ final_rotation.T
            final_angle_error = np.arccos(np.clip(
                (np.trace(final_rotation_error) - 1.0) / 2.0,
                -1.0,
                1.0,
            ))
            #eliminar el converged
            self.robot_data.qpos[:arm_dof_count] = self.current_q[:arm_dof_count]
            mujoco.mj_forward(self.model, self.robot_data)

            #publish the result
            command = np.concatenate((q_copy[:7], [q_copy[7]]))
            if not np.all(np.isfinite(command)):
                self.get_logger().error("IK result is invalid and will not be published.")
                return

            result_msg = JointState()
            result_msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "gripper"]
            data = command.tolist()
            result_msg.position = data
            effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
            result_msg.effort = effort
            self.publisher_result.publish(result_msg)
            self.get_logger().info(f"Published inverse kinematics result: {result_msg.position}")
            
            self.robot_data.qpos[:arm_dof_count] = self.current_q[:arm_dof_count]
            mujoco.mj_forward(self.model, self.robot_data)

            time.sleep(0.01)  # Small delay to ensure the message is sent before the next iteration




    def rpy_to_rotation_matrix(self, rpy):
        roll, pitch, yaw = rpy
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        Rx = np.array([[1, 0, 0],
                       [0, cr, -sr],
                       [0, sr, cr]])
        Ry = np.array([[cp, 0, sp],
                       [0, 1, 0],
                       [-sp, 0, cp]])
        Rz = np.array([[cy, -sy, 0],
                       [sy, cy, 0],
                       [0, 0, 1]])
        R = Rz @ Ry @ Rx
        return R

    def vector_to_skew_symmetric(self, p):
        p = np.asarray(p, dtype=float).flatten()
        return np.array([[0, -p[2], p[1]],
                         [p[2], 0, -p[0]],
                         [-p[1], p[0], 0]])
    def calculate_adj_matrix(self, R, p):
        R = np.asarray(R, dtype=float)
        p_skew = self.vector_to_skew_symmetric(p)
        adj = np.zeros((6, 6))
        adj[0:3, 0:3] = R
        # adj[0:3, 3:6] is already zero
        adj[3:6, 0:3] = p_skew @ R
        adj[3:6, 3:6] = R
        return adj

    # Model Functions
    def _try_assign_model_data(self):
        if self.model is None or self.robot_data is None or self.init_q is None:
            return False

        self.assign_model_data(self.init_q, self.init_vel)
        return True

    def assign_model_data(self, init_q, init_vel):
        if self.model is not None and self.robot_data is not None:
            qpos = np.zeros(self.model.nq)
            qpos[:min(len(init_q), self.model.nq)] = init_q[:self.model.nq]
            qvel = np.zeros(self.model.nv)
            if init_vel is not None:
                qvel[:min(len(init_vel), self.model.nv)] = init_vel[:self.model.nv]

            self.robot_data.qpos[:] = qpos
            self.robot_data.qvel[:] = qvel
            self.body_ids = {
                self.model.body(i).name: self.model.body(i).id
                for i in range(self.model.nbody) if self.model.body(i).name
                            }
            self.ee_site_ids = {
                self.model.site(i).name: self.model.site(i).id
                for i in range(self.model.nsite) if self.model.site(i).name
                               }
            # assign jacobian matrixes
            self.jacp = np.zeros((3, self.model.nv))
            self.jacr = np.zeros((3, self.model.nv))
            self.get_logger().info('Model data assigned successfully.')
            return True

        else:
            self.get_logger().error('Model or robot data is not initialized. Cannot assign data.')
            return False

    def get_model_callback(self, msg):
        if msg.data:
            self.model_path = msg.data
            self.model = mujoco.MjModel.from_xml_path(self.model_path)
            self.robot_data = mujoco.MjData(self.model)
            self.get_logger().info(f'Model path received: {self.model_path}')
            self._try_assign_model_data()

    # def joint_state_callback(self, msg):
    #     # self.get_logger().info(f"Received joint state: {msg.position}")

    #     # extract the joint positions from the message and store them in current_q

    
    
def main(args=None):
    rclpy.init(args=args)
    inverse_kinematics_node = InverseKinematicsNodeBodyJacobian()
    try:
        rclpy.spin(inverse_kinematics_node)
    except KeyboardInterrupt:
        pass
    with inverse_kinematics_node.ik_condition:
        inverse_kinematics_node.stop_ik_thread = True
        inverse_kinematics_node.ik_condition.notify()
    inverse_kinematics_node.ik_thread.join()
    inverse_kinematics_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
# Este nodo debe usar el reloj de la simulacion, con el siguiente parametro
#ros2 run ros2_to_mujoco_nero inverse_kinematics_node --ros-args -p use_sim_time:=true

# to test the comunication only with mujoco node use the following command
"""ros2 topic pub --once /inverse_kinematics_result \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]}"""
