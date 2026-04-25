from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'holoocean_ros2_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.json') + glob('config/*.yaml')),
        # Install RViz config
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
        # Install scripts
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*.sh')),
        # Install web dashboard
        (os.path.join('share', package_name, 'dashboard'),
            glob('dashboard/*.html') + glob('dashboard/*.js') + glob('dashboard/*.css')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='HoloOcean v2.3.0 to ROS2 Humble bridge for surface vessel sonar mapping',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'holoocean_bridge = holoocean_ros2_bridge.holoocean_bridge_node:main',
            'vessel_teleop    = holoocean_ros2_bridge.vessel_teleop_node:main',
            'dashboard_server = holoocean_ros2_bridge.dashboard_server_node:main',
        ],
    },
)
