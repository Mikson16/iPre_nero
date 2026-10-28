#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from typing import Any, cast

from pyAgxArm import (
	AgxArmFactory,
	ArmModel,
	NeroFW,
	create_agx_arm_config,
)


class NeroCalibrationNode(Node):
	def __init__(self):
		super().__init__('nero_calibration_node')

		self.declare_parameter('channel', 'can0')
		self.declare_parameter('bitrate', 1000000)
		self.declare_parameter('firmware', 'v111')
		self.declare_parameter('joint_index', 1)

		self.channel = str(self.get_parameter('channel').value)
		self.bitrate = int(cast(int, self.get_parameter('bitrate').value))
		self.firmware = str(self.get_parameter('firmware').value)
		self.joint_index = int(
			cast(int, self.get_parameter('joint_index').value)
		)

		if self.joint_index not in (1, 2, 3, 4, 5, 6, 7, 255):
			raise ValueError(
				'joint_index must be 1-7, or 255 for all joints'
			)

		self.robot: Any = self._create_robot()
		self.robot.connect()

		if not self.robot.disable(self.joint_index):
			raise RuntimeError('Could not disable the selected joint(s)')

		self.get_logger().info(
			f'Robot connected. Joint {self.joint_index} is disabled.'
		)
		self.get_logger().info(
			'Place the joint(s) at the mechanical zero position, then call '
			'/calibrate_joint.'
		)

		self.calibration_service = self.create_service(
			Trigger,
			'calibrate_joint',
			self.calibrate_callback,
		)

	def _create_robot(self):
		firmware_name = str(self.firmware).upper()
		if firmware_name == 'DEFAULT':
			raise ValueError(
				'Nero calibration requires firmware V111 or newer.'
			)

		try:
			firmware = getattr(NeroFW, firmware_name)
		except AttributeError as error:
			valid_firmware = ('DEFAULT', 'V111', 'V112', 'V120', 'V121')
			raise ValueError(
				f'Unsupported firmware {self.firmware!r}. '
				f'Use one of: {", ".join(valid_firmware)}.'
			) from error

		config = create_agx_arm_config(
			robot=ArmModel.NERO,
			firmeware_version=firmware,
			interface='socketcan',
			channel=self.channel,
			bitrate=self.bitrate,
		)
		return AgxArmFactory.create_arm(config)

	def calibrate_callback(self, request, response):
		del request

		try:
			self.get_logger().warn(
				f'Calibrating joint {self.joint_index} at its current position.'
			)
			self.robot.calibrate_joint(self.joint_index)

			if not self.robot.enable(self.joint_index):
				response.success = False
				response.message = (
					'Calibration command was sent, but the joint could not '
					'be enabled.'
				)
				return response

			response.success = True
			response.message = (
				f'Joint {self.joint_index} calibrated and enabled.'
			)
			self.get_logger().info(response.message)
		except Exception as error:
			response.success = False
			response.message = f'Calibration failed: {error}'
			self.get_logger().error(response.message)

		return response

	def destroy_node(self):
		if hasattr(self, 'robot') and self.robot.is_connected():
			self.robot.disconnect()
		super().destroy_node()


def main(args=None):
	rclpy.init(args=args)
	node = NeroCalibrationNode()

	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()

# Created with help of copilot
