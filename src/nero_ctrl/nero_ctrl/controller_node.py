#!/usr/bin/env python3
import rclpy
import os
import sys
import threading
import mujoco
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool, Float64MultiArray
from sensor_msgs.msg import JointState


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info('Controller node has been started.')

        # atributes

        # qos
        self.node_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        #timers

        # publishers
        self.start_node_publisher = self.create_publisher(Bool, '/start_running_nodes', self.node_qos)

        # subscribers
        self.subscription = self.create_subscription(Bool, '/controller_ready', self.start_node_callback, self.node_qos)

    # callbacks and functions
    def start_node_callback(self, msg):
        if msg.data:
            self.get_logger().info('Starting controller node.')
            msg = Bool()
            msg.data = True
            self.start_node_publisher.publish(msg)

    # define your callbacks and functions here

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
