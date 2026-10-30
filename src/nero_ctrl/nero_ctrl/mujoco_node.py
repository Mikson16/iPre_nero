#!/usr/bin/env python3

import rclpy
import os
import time
import mujoco
import mujoco.viewer
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool, Float64MultiArray
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

"""This node handle the mujoco simulator"""


class MujocoNode(Node):
    def __init__(self):
        super().__init__('mujoco_node')
        self.get_logger().info('Mujoco node has been started.')
        
         # QoS profile 
        latch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                
        # atributes
        self.timer_pr = 0.01 # 100Hz
        self.start_requested = False
        self.ready_sent = False
        self.last_command = None
        # This may change if needed
        self.model_path = os.path.expanduser('~/Projects/iPre_nero/src/models_for_simulation/agilex_nero/scene.xml')
        
        # publishers
        self.publisher_state_ready = self.create_publisher(Bool, '/mujoco_ready', latch_qos)
        self.publisher_joint_state = self.create_publisher(JointState, '/joint_states', latch_qos)
        self.publisher_clock_sim = self.create_publisher(Clock, '/clock', latch_qos)
        
        #subscriptions
        self.subscription = self.create_subscription(
            Bool,
            '/start_running_nodes',
            self.start_running_nodes_callback,
            latch_qos,
        )
        self.subscription = self.create_subscription(Float64MultiArray, '/inverse_kinematics_result', self.inverse_kinematics_result_callback, 10)
    
    def inverse_kinematics_result_callback(self, msg):
        # get the data form the msg
        self.get_logger().info(f"Received inverse kinematics result: {msg.data}")
        command = np.asarray(msg.data, dtype=float)
        if command.shape != (8,) or not np.all(np.isfinite(command)):
            self.get_logger().error('Ignoring invalid IK command; expected 8 finite actuator positions.')
            return
        self.last_command = command
        
        
    def start_running_nodes_callback(self, msg):
        if msg.data and not self.start_requested:
            self.get_logger().info('Starting Mujoco simulator.')
            self.start_requested = True
    
    def update_simulation(self, model, data):
        # apply the last command received from the inverse kinematics node
        if self.last_command is not None:
            num_actuators = min(len(self.last_command), len(data.ctrl))
            data.ctrl[:num_actuators] = self.last_command[:num_actuators]
            
        # update simulation
        mujoco.mj_step(model, data)
        
        # share clock
        clock_msg = Clock()
        # convert units
        sec = int(data.time)
        nanosec = int((data.time - sec) * 1e9)
        clock_msg.clock.sec = sec
        clock_msg.clock.nanosec = nanosec
        self.publisher_clock_sim.publish(clock_msg)
        
        # share joint states
        joint_state_msg = JointState()
        joint_state_msg.position = data.qpos[:7].tolist()
        self.publisher_joint_state.publish(joint_state_msg)
        
        
            
def main(args=None):
    rclpy.init(args=args)
    mujoco_node = MujocoNode()
    try:
        model = mujoco.MjModel.from_xml_path(mujoco_node.model_path)
        data = mujoco.MjData(model)

        mujoco_node.get_logger().info(
            'Waiting for /start_running_nodes...'
        )

        while rclpy.ok() and not mujoco_node.start_requested:
            rclpy.spin_once(mujoco_node, timeout_sec=0.1)

        if not rclpy.ok():
            return

        mujoco_node.get_logger().info(
            'Start message received. Launching MuJoCo.'
        )

        with mujoco.viewer.launch_passive(model, data) as viewer:
            if not mujoco_node.ready_sent:
                mujoco_node.publisher_state_ready.publish(Bool(data=True))
                mujoco_node.ready_sent = True

            while rclpy.ok() and viewer.is_running():
                rclpy.spin_once(mujoco_node, timeout_sec=0.0)

                mujoco_node.update_simulation(model, data)
                viewer.sync()

                time.sleep(model.opt.timestep)
    except Exception as e:
        mujoco_node.get_logger().error(f"Failed to load Mujoco model or launch viewer: {e}")
       
    finally:
        mujoco_node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
    
# Mensaje de prueba para levantar el nodo de manera individual
# ros2 topic pub /start_running_nodes std_msgs/msg/Bool "{data: true}" -1
# publisher: beginning loop
# publishing #1: std_msgs.msg.Bool(data=True)
