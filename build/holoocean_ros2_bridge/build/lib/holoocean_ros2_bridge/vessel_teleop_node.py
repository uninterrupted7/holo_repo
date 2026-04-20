#!/usr/bin/env python3
"""
vessel_teleop_node.py
=====================
Keyboard teleoperation node for the HoloOcean SurfaceVessel.

Publishes geometry_msgs/Twist to /holoocean/cmd_vel

Controls (hold-to-move):
  w / s      → forward / backward (hold to move)
  a / d      → turn left / right (hold to turn)
  SPACE      → stop all movement
  q          → quit

Keys must be held to maintain movement. Releasing a key stops that axis
after a short debounce period. Hold multiple keys to combine motions
(e.g. w+a for forward + left turn).
"""

import os
import select
import sys
import termios
import tty
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP = """
HoloOcean Surface Vessel Teleop (Hold-to-Move)
─────────────────────────────────────────────────
  w / s    forward / backward (hold)
  a / d    turn left / right (hold)
  SPACE    emergency stop
  q        quit
─────────────────────────────────────────────────
"""

DEFAULT_LINEAR_SCALE  = 10.0
DEFAULT_ANGULAR_SCALE = 2.0
KEY_TIMEOUT           = 0.15
POLL_INTERVAL         = 0.05


class VesselTeleopNode(Node):
    def __init__(self):
        super().__init__('vessel_teleop')
        self.declare_parameter('linear_scale',  DEFAULT_LINEAR_SCALE)
        self.declare_parameter('angular_scale', DEFAULT_ANGULAR_SCALE)
        self._linear_scale  = self.get_parameter('linear_scale').value
        self._angular_scale = self.get_parameter('angular_scale').value

        self._pub   = self.create_publisher(Twist, '/holoocean/cmd_vel', 10)
        self._timer = self.create_timer(1.0 / 10.0, self._publish_cmd)

        self._forward = 0.0
        self._turn    = 0.0
        self._last_forward_time = time.time()
        self._last_turn_time    = time.time()
        self._running = True

        self.get_logger().info('Teleop node started. Hold w/a/s/d to move.')
        print(HELP)

    def _publish_cmd(self):
        msg = Twist()
        msg.linear.x  = self._forward
        msg.angular.z = self._turn
        self._pub.publish(msg)

    def _handle_key(self, key):
        if key == 'q':
            self._running = False
            return

        if key == ' ':
            self._forward = 0.0
            self._turn    = 0.0
            self._last_forward_time = time.time()
            self._last_turn_time    = time.time()
            self.get_logger().info('STOP')
            return

        if key == 'w':
            self._forward =  self._linear_scale
        elif key == 's':
            self._forward = -self._linear_scale
        elif key == 'a':
            self._turn =  self._angular_scale
        elif key == 'd':
            self._turn = -self._angular_scale
        else:
            return

        self._last_forward_time = time.time()
        self._last_turn_time    = time.time()

    def _check_timeouts(self):
        now = time.time()
        if now - self._last_forward_time > KEY_TIMEOUT:
            self._forward = 0.0
        if now - self._last_turn_time > KEY_TIMEOUT:
            self._turn = 0.0

    def run(self):
        tty_fd = None
        settings = None

        try:
            tty_fd = os.open('/dev/tty', os.O_RDONLY)
            settings = termios.tcgetattr(tty_fd)
            tty.setraw(tty_fd)
        except (termios.error, OSError):
            if tty_fd is not None:
                os.close(tty_fd)
            self.get_logger().warn('No TTY available. Teleop keyboard disabled.')
            self.get_logger().info('Use "ros2 topic pub /holoocean/cmd_vel geometry_msgs/msg/Twist" to send commands.')
            while rclpy.ok() and self._running:
                rclpy.spin_once(self, timeout_sec=1.0)
            return

        try:
            while rclpy.ok() and self._running:
                rlist, _, _ = select.select([tty_fd], [], [], POLL_INTERVAL)
                if rlist:
                    key = os.read(tty_fd, 1).decode('utf-8', errors='ignore')
                    self._handle_key(key)
                self._check_timeouts()
                rclpy.spin_once(self, timeout_sec=0.0)

        except KeyboardInterrupt:
            pass
        finally:
            if settings is not None:
                termios.tcsetattr(tty_fd, termios.TCSADRAIN, settings)
            os.close(tty_fd)
            self._pub.publish(Twist())
            self.get_logger().info('Teleop stopped.')

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VesselTeleopNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
