"""
Tests for vessel_teleop_node.py
"""

import time
import unittest
from unittest.mock import patch, MagicMock

import rclpy
from geometry_msgs.msg import Twist


class TestVesselTeleopNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_default_parameters(self):
        from holoocean_ros2_bridge.vessel_teleop_node import VesselTeleopNode
        node = VesselTeleopNode()
        self.assertAlmostEqual(
            node.get_parameter('linear_scale').value, 1.0)
        self.assertAlmostEqual(
            node.get_parameter('angular_scale').value, 0.5)
        node.destroy_node()

    def test_custom_parameters(self):
        from holoocean_ros2_bridge.vessel_teleop_node import VesselTeleopNode
        from rclpy.parameter import Parameter
        node = VesselTeleopNode()
        node.set_parameters([
            Parameter('linear_scale', Parameter.Type.DOUBLE, 2.0),
            Parameter('angular_scale', Parameter.Type.DOUBLE, 1.0),
        ])
        self.assertAlmostEqual(
            node.get_parameter('linear_scale').value, 2.0)
        self.assertAlmostEqual(
            node.get_parameter('angular_scale').value, 1.0)
        node.destroy_node()

    def test_forward_key_sets_positive_linear(self):
        from holoocean_ros2_bridge.vessel_teleop_node import VesselTeleopNode
        node = VesselTeleopNode()
        node._forward = 0.0
        node._handle_key('w')
        self.assertAlmostEqual(node._forward, 1.0)
        node.destroy_node()

    def test_backward_key_sets_negative_linear(self):
        from holoocean_ros2_bridge.vessel_teleop_node import VesselTeleopNode
        node = VesselTeleopNode()
        node._forward = 0.0
        node._handle_key('s')
        self.assertAlmostEqual(node._forward, -1.0)
        node.destroy_node()

    def test_left_key_sets_positive_angular(self):
        from holoocean_ros2_bridge.vessel_teleop_node import VesselTeleopNode
        node = VesselTeleopNode()
        node._turn = 0.0
        node._handle_key('a')
        self.assertAlmostEqual(node._turn, 0.5)
        node.destroy_node()

    def test_right_key_sets_negative_angular(self):
        from holoocean_ros2_bridge.vessel_teleop_node import VesselTeleopNode
        node = VesselTeleopNode()
        node._turn = 0.0
        node._handle_key('d')
        self.assertAlmostEqual(node._turn, -0.5)
        node.destroy_node()

    def test_space_key_stops_all_movement(self):
        from holoocean_ros2_bridge.vessel_teleop_node import VesselTeleopNode
        node = VesselTeleopNode()
        node._forward = 1.0
        node._turn = 0.5
        node._handle_key(' ')
        self.assertAlmostEqual(node._forward, 0.0)
        self.assertAlmostEqual(node._turn, 0.0)
        node.destroy_node()

    def test_timeout_stops_movement(self):
        from holoocean_ros2_bridge.vessel_teleop_node import VesselTeleopNode, KEY_TIMEOUT
        node = VesselTeleopNode()
        node._forward = 1.0
        node._last_forward_time = time.time() - KEY_TIMEOUT - 0.1
        node._turn = 0.5
        node._last_turn_time = time.time() - KEY_TIMEOUT - 0.1
        node._check_timeouts()
        self.assertAlmostEqual(node._forward, 0.0)
        self.assertAlmostEqual(node._turn, 0.0)
        node.destroy_node()

    def test_publish_cmd_publishes_twist(self):
        from holoocean_ros2_bridge.vessel_teleop_node import VesselTeleopNode
        node = VesselTeleopNode()
        node._forward = 1.0
        node._turn = 0.5
        msgs = []

        class FakePublisher:
            def publish(self, msg):
                msgs.append(msg)

        node._pub = FakePublisher()
        node._publish_cmd()
        self.assertEqual(len(msgs), 1)
        self.assertAlmostEqual(msgs[0].linear.x, 1.0)
        self.assertAlmostEqual(msgs[0].angular.z, 0.5)
        node.destroy_node()


if __name__ == '__main__':
    unittest.main()
