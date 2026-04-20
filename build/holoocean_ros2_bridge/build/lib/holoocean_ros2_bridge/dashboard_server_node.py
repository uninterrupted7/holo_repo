#!/usr/bin/env python3
"""
dashboard_server_node.py
========================
Lightweight ROS2 node that serves the ASV web dashboard on http://localhost:8080.

Run this alongside rosbridge_server to get full live data in the browser:
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    ros2 run holoocean_ros2_bridge dashboard_server
"""

import os
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class DashboardHandler(SimpleHTTPRequestHandler):
    """Serve files from the package's dashboard directory."""
    dashboard_dir: str = ''

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=self.dashboard_dir, **kwargs)

    def log_message(self, fmt, *args):
        pass  # suppress HTTP access log noise


class DashboardServerNode(Node):
    def __init__(self):
        super().__init__('dashboard_server')

        self.declare_parameter('port', 8080)
        self.declare_parameter('host', '0.0.0.0')

        port = self.get_parameter('port').value
        host = self.get_parameter('host').value

        # Locate the dashboard HTML
        pkg_share = get_package_share_directory('holoocean_ros2_bridge')
        dash_dir  = os.path.join(pkg_share, 'dashboard')

        if not os.path.isdir(dash_dir):
            self.get_logger().error(
                f'Dashboard directory not found: {dash_dir}\n'
                'Make sure the package was built with setup.py including the dashboard files.'
            )
            return

        DashboardHandler.dashboard_dir = dash_dir

        self._server = HTTPServer((host, port), DashboardHandler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'ASV Dashboard available at  http://localhost:{port}/asv_dashboard.html\n'
            f'  (Serving from {dash_dir})\n'
            '  Requires rosbridge_server on ws://localhost:9090'
        )

    def destroy_node(self):
        if hasattr(self, '_server'):
            self._server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
