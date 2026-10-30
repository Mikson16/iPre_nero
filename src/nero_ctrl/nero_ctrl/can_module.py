#!/usr/bin/env python3
import can
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, PoseArray

"""This node has to communicate with the CAN bus of agx_arm_ws """

class CanModule(Node):
    def __init__(self):
        super().__init__('can_module')
        self.get_logger().info("CAN module initialized.")

        # parameters
        self._joint_state = JointState()
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        self.gripper_name = "gripper"
        self.init_pose = [0.0, -0.40, 0.0, 2.0, 0.0, 0.0, 0.0, 0.05]
        self.init_pose_ready = False
        self.start_requested = False
        self.feedback_ready = False
        self.initial_pose_attempts = 0
        self.initial_pose_first_sent_at = None


        # QoS settings
        self.init_node_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        ik_qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
        # timers

        # publishers
        # publishers of agx_arm_ctrl_single_node.py
        self.init_state_joint_pub = self.create_publisher(JointState, 'control/move_j', self.init_node_qos)
        self.move_j_pub = self.create_publisher(JointState, "control/move_j", 10)
        self.move_p_pub = self.create_publisher(PoseStamped, "control/move_p", 10)
        self.move_l_pub = self.create_publisher(PoseStamped, "control/move_l", 10)
        self.move_c_pub = self.create_publisher(PoseArray, "control/move_c", 10)
        self.move_js_pub = self.create_publisher(JointState, "control/move_js", 10)
        
        # others publishers (can modify and use this)
        self.start_node_publisher = self.create_publisher(Bool, '/start_running_nodes', self.init_node_qos)
        
        self.joint_state_publisher_for_ik = self.create_publisher(
            JointState,
            '/ik/joint_states',
            ik_qos,
        )
        
        # subscribers
        self.create_subscription(Float64MultiArray, 'test_pos', self.test_pos_callback, 10)
        self.create_subscription(Bool, 'start_running_nodes', self.start_node_callback, 10)
        self.create_subscription(JointState, 'feedback/joint_states', self.joint_states_callback, ik_qos)
        self.create_subscription(JointState, 'ik/ik_result', self.ik_result_callback, ik_qos)
        self.initial_pose_timer = self.create_timer(0.25, self._publish_initial_pose_when_ready)


    def test_pos_callback(self, msg):
        data = msg.data
        if data and data[0] == 1.0:
            self.get_logger().info("Test position 1 received.")
            self._joint_state.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "gripper"]
            self._joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

        else:
            self.get_logger().info("Test position 2 received.")
            self._joint_state.name = ["joint1", "joint2", "joint3","joint4", "joint5", "joint6", "joint7", "gripper"]
            self._joint_state.position = [0.5, 0.5, 0.5, 0.5, 10.0, 0.5, 0.5, 0.05]
            effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

        msg = JointState()
        msg.name = self._joint_state.name
        msg.position = self._joint_state.position
        msg.effort = effort
        self.move_j_pub.publish(msg)
        self.get_logger().info(f"Published joint states: {msg.position}")

    def start_node_callback(self, msg):
        if msg.data and not self.start_requested:
            self.start_requested = True
            self.get_logger().info('Starting CAN module.')
            self.start_node_publisher.publish(Bool(data=True))

    def _publish_initial_pose_when_ready(self):
        if not self.start_requested or not self.feedback_ready:
            return

        now = time.monotonic()
        if self.initial_pose_first_sent_at is None:
            self.initial_pose_first_sent_at = now
            return

        if now - self.initial_pose_first_sent_at < 1.0:
            return
        if self.initial_pose_attempts >= 3:
            self.initial_pose_timer.cancel()
            return
        if self.initial_pose_attempts and now - self.initial_pose_first_sent_at < 0.5 * (self.initial_pose_attempts + 1):
            return

        msg = JointState()
        msg.name = self.joint_names + [self.gripper_name]
        msg.position = self.init_pose
        msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.init_state_joint_pub.publish(msg)
        self.initial_pose_attempts += 1
        self.init_pose_ready = True
        self.get_logger().info(
            f"Published initial joint states (attempt {self.initial_pose_attempts}/3): {msg.position}"
        )

    
    def joint_states_callback(self, msg):
        if len(msg.position) >= len(self.joint_names):
            self.feedback_ready = True
        if self.init_pose_ready:
            self._joint_state.name = msg.name
            self._joint_state.position = msg.position
            self.joint_state_publisher_for_ik.publish(self._joint_state)

    def ik_result_callback(self, msg):
        self.get_logger().info(f"Received IK result: {msg.position}")
        # initially only with movej
        self.move_j_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    can_module = CanModule()
    rclpy.spin(can_module)
    can_module.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
