import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/asemiz/ros2_ws/install/holoocean_ros2_bridge'
