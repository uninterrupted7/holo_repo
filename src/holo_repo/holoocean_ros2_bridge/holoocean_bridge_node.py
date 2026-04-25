#!/usr/bin/env python3
"""
holoocean_bridge_node.py
========================
HoloOcean v2.3.0 → ROS2 Humble Bridge Node  (Fossen Dynamics Edition)

Runs a SurfaceVessel with ProfilingSonar (multibeam-equivalent) and
publishes all sensor data as standard ROS2 messages.

Published Topics:
  /holoocean/sonar/points          [sensor_msgs/PointCloud2]   - Multibeam sonar 3D points
  /holoocean/sonar/image           [sensor_msgs/Image]         - Raw sonar intensity image
  /holoocean/imu                   [sensor_msgs/Imu]
  /holoocean/gps                   [sensor_msgs/NavSatFix]
  /holoocean/dvl/velocity          [geometry_msgs/TwistStamped]
  /holoocean/odom                  [nav_msgs/Odometry]         - Ground-truth pose
  /holoocean/pose                  [geometry_msgs/PoseStamped] - Ground-truth pose (simple)

Subscribed Topics:
  /holoocean/cmd_vel               [geometry_msgs/Twist]       - Vessel velocity command
"""

import json
import math
import os
import struct
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ROS2 message types
from builtin_interfaces.msg import Time
from geometry_msgs.msg import (
    PoseStamped, TwistStamped, Twist,
    Point, Quaternion, Vector3, TransformStamped
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import (
    Imu, NavSatFix, NavSatStatus,
    PointCloud2, PointField, Image
)
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import TransformBroadcaster

import holoocean

from .fossen_dynamics import FossenASVModel


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Euler angles (radians) to geometry_msgs/Quaternion."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def rotation_matrix_to_quaternion(R: np.ndarray) -> Quaternion:
    """Convert a 3x3 rotation matrix to geometry_msgs/Quaternion."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = Quaternion()
    q.w, q.x, q.y, q.z = float(w), float(x), float(y), float(z)
    return q


def make_header(node: Node, frame_id: str) -> Header:
    """Create a std_msgs/Header with current ROS time."""
    header = Header()
    header.stamp = node.get_clock().now().to_msg()
    header.frame_id = frame_id
    return header


def sonar_to_pointcloud2(
    intensities: np.ndarray,
    azimuth_deg: float,
    range_min: float,
    range_max: float,
    elevation_deg: float,
    intensity_threshold: float,
    header: Header
) -> PointCloud2:
    """
    Convert ProfilingSonar output to sensor_msgs/PointCloud2.

    The ProfilingSonar returns a 2D array [RangeBins x AzimuthBins] of
    intensity values. We transpose to [AzimuthBins x RangeBins] for processing.
    We perform peak detection per azimuth beam to find the strongest range
    return, then project that into 3D.

    Coordinate convention (sensor frame):
      - x: forward (along vessel heading)
      - y: starboard (right)
      - z: downward (positive toward seafloor)

    The sonar fans out in the y-z plane (across-track) from the vessel keel.
    """
    n_range, n_azimuth = intensities.shape
    intensities = intensities.T
    azimuth_rad = math.radians(azimuth_deg)
    elevation_rad = math.radians(elevation_deg)
    range_bins = n_range

    # Beam angle for each azimuth bin (centered fan, e.g. -60° to +60°)
    azimuth_angles = np.linspace(
        -azimuth_rad / 2.0, azimuth_rad / 2.0, n_azimuth
    )

    # Range value for each range bin
    ranges = np.linspace(range_min, range_max, range_bins)

    points = []

    for az_idx in range(n_azimuth):
        beam = intensities[az_idx, :]  # intensity across range bins

        if beam.max() < intensity_threshold:
            continue

        # Peak detection: find the first peak above threshold
        peak_idx = int(np.argmax(beam))
        r = ranges[peak_idx]
        intensity_val = float(beam[peak_idx])

        az = azimuth_angles[az_idx]

        # Project: sonar points downward in ENU frame
        # ENU: +z = up, so seafloor is negative z
        # HoloOcean: +z = down (toward seafloor)
        # Convert: negate z to get proper ENU coordinates
        x = 0.0
        y = r * math.sin(az)              # across-track (port/starboard)
        z = -r * math.cos(az)             # depth (negative = below vessel in ENU)

        points.append((x, y, z, intensity_val))

    # Build PointCloud2 message
    fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    point_step = 16  # 4 × float32
    data = bytearray()
    for (px, py, pz, pi) in points:
        data += struct.pack('ffff', px, py, pz, pi)

    pc2 = PointCloud2()
    pc2.header = header
    pc2.height = 1
    pc2.width = len(points)
    pc2.fields = fields
    pc2.is_bigendian = False
    pc2.point_step = point_step
    pc2.row_step = point_step * len(points)
    pc2.data = bytes(data)
    pc2.is_dense = True
    return pc2


def _numpy_resize(arr: np.ndarray, new_h: int, new_w: int) -> np.ndarray:
    """
    Resize a 2-D array to (new_h, new_w) using nearest-neighbour sampling.
    Pure numpy — no OpenCV dependency required.
    """
    old_h, old_w = arr.shape[:2]
    row_idx = (np.arange(new_h) * old_h / new_h).astype(int)
    col_idx = (np.arange(new_w) * old_w / new_w).astype(int)
    row_idx = np.clip(row_idx, 0, old_h - 1)
    col_idx = np.clip(col_idx, 0, old_w - 1)
    return arr[np.ix_(row_idx, col_idx)]


def sonar_intensities_to_image(intensities: np.ndarray, header: Header) -> Image:
    """
    Convert ProfilingSonar raw 2D intensity array to sensor_msgs/Image (mono8).
    Legacy function kept for backward compatibility.
    """
    norm = intensities.astype(np.float32)
    if norm.max() > 0:
        norm = (norm / norm.max() * 255.0).astype(np.uint8)
    else:
        norm = np.zeros_like(norm, dtype=np.uint8)

    img = Image()
    img.header = header
    img.height = int(intensities.shape[0])
    img.width = int(intensities.shape[1])
    img.encoding = 'mono8'
    img.is_bigendian = 0
    img.step = img.width
    img.data = norm.flatten().tobytes()
    return img


def profiling_sonar_to_image(
    intensities: np.ndarray,
    header: Header,
    out_width: int = 800,
    out_height: int = 600,
    colormap: str = 'gray',
    log_scale: bool = True,
    dynamic_range_db: float = 40.0,
) -> Image:
    """
    Convert ProfilingSonar raw 2D intensity array to a colour-mapped,
    resized sensor_msgs/Image for clear obstacle visualisation.

    Processing pipeline:
      1. Log-scale compression (20·log10) to bring out weak returns
      2. Normalise to [0, 255] within the specified dynamic range
      3. Resize to (out_height × out_width) using nearest-neighbour
      4. Apply colormap (copper / sonar / gray)

    Parameters
    ----------
    intensities : np.ndarray
        Raw 2-D intensity array from ProfilingSonar (RangeBins × AzimuthBins).
    header : Header
        ROS header.
    out_width : int
        Output image width in pixels (default 800).
    out_height : int
        Output image height in pixels (default 600).
    colormap : str
        'copper', 'sonar', or 'gray'.
    log_scale : bool
        Apply 20·log10 compression (recommended).
    dynamic_range_db : float
        dB window to display.

    Returns
    -------
    sensor_msgs/Image
        rgb8 (colour) or mono8 (gray) encoded image.
    """
    wf = intensities.astype(np.float64).copy()

    # ── 1. Log-scale compression ──────────────────────────────────────
    if log_scale:
        floor = 1e-10
        wf = np.clip(wf, floor, None)
        wf = 20.0 * np.log10(wf)
        peak = wf.max()
        wf = np.clip(wf, peak - dynamic_range_db, peak)

    # ── 2. Normalise to 0-255 ─────────────────────────────────────────
    wf_min, wf_max = wf.min(), wf.max()
    if wf_max > wf_min:
        norm = ((wf - wf_min) / (wf_max - wf_min) * 255.0)
    else:
        norm = np.zeros_like(wf)
    indices = norm.astype(np.uint8)

    # ── 3. Resize to output dimensions ────────────────────────────────
    if indices.shape[0] != out_height or indices.shape[1] != out_width:
        indices = _numpy_resize(indices, out_height, out_width)

    # ── 4. Apply colourmap ────────────────────────────────────────────
    if colormap == 'gray':
        img = Image()
        img.header = header
        img.height, img.width = indices.shape
        img.encoding = 'mono8'
        img.is_bigendian = 0
        img.step = img.width
        img.data = indices.flatten().tobytes()
        return img

    lut = _COPPER_LUT if colormap == 'copper' else _SONAR_LUT
    rgb = lut[indices]  # (h, w, 3)

    img = Image()
    img.header = header
    img.height, img.width = indices.shape
    img.encoding = 'rgb8'
    img.is_bigendian = 0
    img.step = img.width * 3
    img.data = rgb.astype(np.uint8).flatten().tobytes()
    return img


# ── Sidescan Sonar Colormap & Waterfall Rendering ─────────────────────────

def _build_copper_lut() -> np.ndarray:
    """
    Build a 256×3 uint8 look-up table that replicates matplotlib's 'copper'
    colormap.  This is the same palette HoloOcean uses in its own examples.

    Copper cmap definition (matplotlib):
        R = min(1.0, t * 1.25)
        G = t * 0.7812
        B = t * 0.4975
    where t = index / 255.
    """
    t = np.linspace(0.0, 1.0, 256, dtype=np.float32)
    r = np.clip(t * 1.25, 0.0, 1.0)
    g = t * 0.7812
    b = t * 0.4975
    lut = np.stack([r, g, b], axis=-1)  # (256, 3)
    return (lut * 255.0).astype(np.uint8)


_COPPER_LUT = _build_copper_lut()  # computed once at import time


def _build_sonar_lut() -> np.ndarray:
    """
    Build a 256×3 uint8 LUT matching real sidescan sonar equipment display.
    Dark seabed → warm amber/yellow highlights, similar to Edgetech / Klein
    paper-chart style.
    """
    t = np.linspace(0.0, 1.0, 256, dtype=np.float32)
    # Deep amber: dark brown → warm yellow-white
    r = np.clip(t * 1.1, 0.0, 1.0)
    g = np.clip(t * 0.85, 0.0, 1.0)
    b = np.clip(t * 0.45, 0.0, 1.0)
    lut = np.stack([r, g, b], axis=-1)
    return (lut * 255.0).astype(np.uint8)


_SONAR_LUT = _build_sonar_lut()


def sidescan_to_waterfall_image(
    waterfall: np.ndarray,
    header: Header,
    colormap: str = 'copper',
    log_scale: bool = True,
    tvg_alpha: float = 0.02,
    dynamic_range_db: float = 40.0,
) -> Image:
    """
    Convert a sidescan sonar waterfall buffer to a colour-mapped
    sensor_msgs/Image that looks like real sidescan sonar output.

    Processing pipeline (matches real SSS processors):
      1. Time-Varying Gain (TVG): compensate range-dependent attenuation
      2. Log-scale compression: 20·log10 to compress dynamic range
      3. Normalise to [0, 255] within the specified dynamic range
      4. Apply colormap LUT (copper = HoloOcean style, sonar = real SSS style)

    Parameters
    ----------
    waterfall : np.ndarray
        2-D float array (rows × range_bins).  Each row is one ping.
        For true SSS display the columns should be arranged:
        [port_far ... port_near | stbd_near ... stbd_far]
        (nadir at centre).
    header : Header
        ROS header to attach.
    colormap : str
        'copper'  – warm copper tones (HoloOcean / matplotlib default)
        'sonar'   – amber/yellow tones (realistic SSS equipment look)
        'gray'    – simple grayscale
    log_scale : bool
        Apply 20·log10 compression (recommended for sonar data).
    tvg_alpha : float
        TVG gain factor.  Larger = more boost for far-range samples.
        Set to 0.0 to disable TVG.
    dynamic_range_db : float
        How many dB of dynamic range to display (clips values below
        max_dB - dynamic_range_db).

    Returns
    -------
    sensor_msgs/Image
        rgb8 encoded colour image ready for RViz2 display.
    """
    wf = waterfall.astype(np.float64).copy()

    # ── 1. Time-Varying Gain ──────────────────────────────────────────
    if tvg_alpha > 0.0:
        n_bins = wf.shape[1]
        # Range index (distance from nadir at centre)
        centre = n_bins // 2
        range_idx = np.abs(np.arange(n_bins) - centre).astype(np.float64)
        # TVG: multiply each column by (1 + alpha * range_index)
        tvg = 1.0 + tvg_alpha * range_idx
        wf *= tvg[np.newaxis, :]

    # ── 2. Log-scale compression ──────────────────────────────────────
    if log_scale:
        # Avoid log(0)
        floor = 1e-10
        wf = np.clip(wf, floor, None)
        wf = 20.0 * np.log10(wf)
        # Clip dynamic range relative to the current maximum
        peak = wf.max()
        wf = np.clip(wf, peak - dynamic_range_db, peak)

    # ── 3. Normalise to 0-255 ─────────────────────────────────────────
    wf_min, wf_max = wf.min(), wf.max()
    if wf_max > wf_min:
        norm = ((wf - wf_min) / (wf_max - wf_min) * 255.0)
    else:
        norm = np.zeros_like(wf)
    indices = norm.astype(np.uint8)  # (rows, cols)

    # ── 4. Apply colourmap ────────────────────────────────────────────
    if colormap == 'gray':
        # Output mono8 for simple grayscale
        img = Image()
        img.header = header
        img.height, img.width = indices.shape
        img.encoding = 'mono8'
        img.is_bigendian = 0
        img.step = img.width
        img.data = indices.flatten().tobytes()
        return img

    lut = _COPPER_LUT if colormap == 'copper' else _SONAR_LUT
    rgb = lut[indices]  # (rows, cols, 3)

    # ── 5. Draw nadir line (thin dark vertical stripe at centre) ──────
    centre_col = rgb.shape[1] // 2
    half_w = max(1, rgb.shape[1] // 200)  # ~0.5% of width
    c_lo = max(0, centre_col - half_w)
    c_hi = min(rgb.shape[1], centre_col + half_w + 1)
    rgb[:, c_lo:c_hi, :] = 0  # dark nadir line

    # ── 6. Build sensor_msgs/Image (rgb8) ─────────────────────────────
    img = Image()
    img.header = header
    img.height, img.width = indices.shape
    img.encoding = 'rgb8'
    img.is_bigendian = 0
    img.step = img.width * 3
    img.data = rgb.astype(np.uint8).flatten().tobytes()
    return img


class HoloOceanBridgeNode(Node):
    """
    ROS2 Node that runs HoloOcean simulation and bridges all sensor
    data to standard ROS2 message types.
    """

    def __init__(self):
        super().__init__('holoocean_bridge')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('scenario_file', '')
        self.declare_parameter('scenario_name', '')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('vessel_frame', 'base_link')
        self.declare_parameter('sonar_frame', 'sonar_link')
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('gps_frame', 'gps_link')
        self.declare_parameter('lat_origin', 0.0)
        self.declare_parameter('lon_origin', 0.0)
        self.declare_parameter('sonar_intensity_threshold', 0.1)
        # Sonar configuration (must match scenario)
        self.declare_parameter('sonar_azimuth_deg', 120.0)
        self.declare_parameter('sonar_elevation_deg', 1.0)
        self.declare_parameter('sonar_range_min', 0.5)
        self.declare_parameter('sonar_range_max', 50.0)
        # Fossen dynamics parameters
        self.declare_parameter('fossen_surge_gain', 50.0)   # [N per unit cmd_vel.linear.x]
        self.declare_parameter('fossen_yaw_gain',   20.0)   # [N·m per unit cmd_vel.angular.z]
        self.declare_parameter('fossen_mass',       50.0)   # [kg]
        self.declare_parameter('fossen_iz',         12.0)   # [kg·m²]
        self.declare_parameter('thrust_scale',      50.0)   # legacy, kept for compat

        scenario_file = self.get_parameter('scenario_file').value
        scenario_name = self.get_parameter('scenario_name').value

        self.world_frame  = self.get_parameter('world_frame').value
        self.vessel_frame = self.get_parameter('vessel_frame').value
        self.sonar_frame  = self.get_parameter('sonar_frame').value
        self.imu_frame    = self.get_parameter('imu_frame').value
        self.gps_frame    = self.get_parameter('gps_frame').value
        self.lat_origin   = self.get_parameter('lat_origin').value
        self.lon_origin   = self.get_parameter('lon_origin').value
        self.intensity_th = self.get_parameter('sonar_intensity_threshold').value
        self.azimuth_deg  = self.get_parameter('sonar_azimuth_deg').value
        self.elevation_deg= self.get_parameter('sonar_elevation_deg').value
        self.range_min    = self.get_parameter('sonar_range_min').value
        self.range_max    = self.get_parameter('sonar_range_max').value
        self.thrust_scale = self.get_parameter('thrust_scale').value
        self._surge_gain  = self.get_parameter('fossen_surge_gain').value
        self._yaw_gain    = self.get_parameter('fossen_yaw_gain').value

        # ── HoloOcean Environment ──────────────────────────────────────────
        self.get_logger().info('Initialising HoloOcean environment...')
        try:
            if scenario_file and os.path.exists(scenario_file):
                self.get_logger().info(f'Loading scenario from file: {scenario_file}')
                with open(scenario_file, 'r') as f:
                    scenario_cfg = json.load(f)
                
                # Dynamically extract sonar configs to overwrite launch parameters
                for agent in scenario_cfg.get('agents', []):
                    for sensor in agent.get('sensors', []):
                        if sensor.get('sensor_type') == 'ProfilingSonar':
                            cfg = sensor.get('configuration', {})
                            if 'RangeMax' in cfg:
                                self.range_max = float(cfg['RangeMax'])
                                self.get_logger().info(f"Overriding sonar_range_max with {self.range_max} from scenario")
                            if 'RangeMin' in cfg:
                                self.range_min = float(cfg['RangeMin'])
                            if 'Azimuth' in cfg:
                                self.azimuth_deg = float(cfg['Azimuth'])
                            if 'Elevation' in cfg:
                                self.elevation_deg = float(cfg['Elevation'])

                self.env = holoocean.make(scenario_cfg=scenario_cfg)
            elif scenario_name:
                self.get_logger().info(f'Loading built-in scenario: {scenario_name}')
                self.env = holoocean.make(scenario_name=scenario_name)
            else:
                raise ValueError('Either scenario_file or scenario_name must be specified!')
            self.get_logger().info('HoloOcean environment ready.')
        except Exception as e:
            self.get_logger().error(f'Failed to create HoloOcean env: {e}')
            raise

        # ── Fossen model ───────────────────────────────────────────────────
        self.fossen = FossenASVModel(
            mass=self.get_parameter('fossen_mass').value,
            Iz=self.get_parameter('fossen_iz').value,
            thruster_separation=0.6,
            max_thrust_per_thruster=self.thrust_scale * 2.0,
        )
        self._fossen_dt = 1.0 / 30.0   # matches ticks_per_sec in scenario
        self._fossen_tau = np.zeros(3, dtype=float)  # last commanded forces

        # ── Command state ──────────────────────────────────────────────────
        # SurfaceVessel control scheme 0: [left_thrust, right_thrust]
        # Use float32 for holoocean compatibility
        self._command = np.array([0.0, 0.0], dtype=np.float32)
        self._last_cmd = np.array([0.0, 0.0], dtype=np.float32)
        self._last_cmd_vel_time = 0.0
        self._cmd_vel_timeout = 1.0

        # ── QoS ───────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Publishers ────────────────────────────────────────────────────
        self.pub_sonar_pc2   = self.create_publisher(PointCloud2,    '/holoocean/sonar/points',   sensor_qos)
        self.pub_sonar_img   = self.create_publisher(Image,          '/holoocean/sonar/image',    sensor_qos)
        self.pub_imu         = self.create_publisher(Imu,            '/holoocean/imu',            sensor_qos)
        self.pub_gps         = self.create_publisher(NavSatFix,      '/holoocean/gps',            sensor_qos)
        self.pub_dvl         = self.create_publisher(TwistStamped,   '/holoocean/dvl/velocity',   sensor_qos)
        self.pub_odom        = self.create_publisher(Odometry,       '/holoocean/odom',           10)
        self.pub_pose        = self.create_publisher(PoseStamped,    '/holoocean/pose',           10)
        self.pub_vessel_marker = self.create_publisher(Marker,       '/holoocean/vessel_marker',  10)

        # Fossen dynamics state publishers (for dashboard / analysis)
        self.pub_fossen_vel  = self.create_publisher(TwistStamped,  '/holoocean/fossen/body_vel',   10)
        self.pub_fossen_force= self.create_publisher(TwistStamped,  '/holoocean/fossen/forces',     10)

        self.pub_sidescan_img = self.create_publisher(Image,         '/holoocean/sidescan/image', sensor_qos)
        self._sidescan_waterfall = None
        self._waterfall_rows = 500

        # Sidescan display parameters
        self.declare_parameter('sidescan_colormap', 'copper')     # 'copper', 'sonar', 'gray'
        self.declare_parameter('sidescan_log_scale', True)
        self.declare_parameter('sidescan_tvg_alpha', 0.02)
        self.declare_parameter('sidescan_dynamic_range_db', 40.0)
        self._sss_colormap  = self.get_parameter('sidescan_colormap').value
        self._sss_log_scale = self.get_parameter('sidescan_log_scale').value
        self._sss_tvg_alpha = self.get_parameter('sidescan_tvg_alpha').value
        self._sss_dyn_range = self.get_parameter('sidescan_dynamic_range_db').value

        # Profiling sonar display parameters
        self.declare_parameter('profiling_sonar_width', 800)
        self.declare_parameter('profiling_sonar_height', 600)
        self.declare_parameter('profiling_sonar_colormap', 'gray')  # 'gray', 'copper', 'sonar'
        self.declare_parameter('profiling_sonar_log_scale', True)
        self.declare_parameter('profiling_sonar_dynamic_range_db', 40.0)
        self._ps_width    = self.get_parameter('profiling_sonar_width').value
        self._ps_height   = self.get_parameter('profiling_sonar_height').value
        self._ps_colormap = self.get_parameter('profiling_sonar_colormap').value
        self._ps_log_scale = self.get_parameter('profiling_sonar_log_scale').value
        self._ps_dyn_range = self.get_parameter('profiling_sonar_dynamic_range_db').value

        # ── Subscriber ────────────────────────────────────────────────────
        self.sub_cmd = self.create_subscription(
            Twist,
            '/holoocean/cmd_vel',
            self._cmd_vel_callback,
            10
        )

        # ── TF Broadcaster ────────────────────────────────────────────────
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── Simulation loop timer (runs at ~100 Hz to prevent bottlenecking) ──────────
        self._timer = self.create_timer(0.01, self._sim_step)

        # ── Autonomous waypoint navigation ─────────────────────────────────
        self.declare_parameter('auto_mode', False)
        self._auto_mode = self.get_parameter('auto_mode').value
        self._auto_start_time = self.get_clock().now()

        # Lawnmower mapping pattern (100x100m area, 25m swath overlap)
        self._waypoints = [
            (25.0, 0.0),      # Start
            (125.0, 0.0),     # Forward 100m
            (125.0, 25.0),    # Turn left 25m
            (25.0, 25.0),     # Loop back 100m
            (25.0, 50.0),     # Turn right 25m
            (125.0, 50.0),    # Forward 100m
            (125.0, 75.0),    # Turn left 25m
            (25.0, 75.0),     # Loop back 100m
            (25.0, 100.0),    # Turn right 25m
            (125.0, 100.0),   # Forward 100m
            (125.0, 0.0),     # Return to start quadrant
        ]
        self._current_wp = 1
        self._wp_reach_tol = 4.0

        self._current_x = 0.0
        self._current_y = 0.0
        self._current_yaw = 0.0

        self._max_thrust = 200.0
        self._kp_heading = 3.0
        self._kp_dist = 6.0

        self.get_logger().info(
            'HoloOcean ROS2 Bridge started.\n'
            f'  Sonar  → /holoocean/sonar/points  (sensor_msgs/PointCloud2)\n'
            f'  SideScan→ /holoocean/sidescan/image  (sensor_msgs/Image)\n'
            f'  IMU    → /holoocean/imu            (sensor_msgs/Imu)\n'
            f'  GPS    → /holoocean/gps            (sensor_msgs/NavSatFix)\n'
            f'  DVL    → /holoocean/dvl/velocity   (geometry_msgs/TwistStamped)\n'
            f'  Odom   → /holoocean/odom           (nav_msgs/Odometry)\n'
            f'  CmdVel ← /holoocean/cmd_vel        (geometry_msgs/Twist)\n'
        )

    # ──────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────

    def _cmd_vel_callback(self, msg: Twist):
        """
        Convert Twist to differential thrust via Fossen 3-DOF dynamics.

        Pipeline:
          cmd_vel → generalised forces τ=[X,0,N]
                  → Fossen integration (M ν̇ + C(ν)ν + D(ν)ν = τ)
                  → thruster allocation → [T_port, T_stbd]

        cmd_vel.linear.x  ∈ [-1, 1]  → surge force X  [N]
        cmd_vel.angular.z ∈ [-1, 1]  → yaw moment  N  [N·m]
        """
        tau = self.fossen.cmd_vel_to_tau(
            msg.linear.x, msg.angular.z,
            surge_gain=self._surge_gain,
            yaw_gain=self._yaw_gain,
        )
        self._fossen_tau = tau

        # Integrate Fossen model to update body velocities
        self.fossen.step(tau, self._fossen_dt)

        # Allocate to thrusters
        T_port, T_stbd = self.fossen.tau_to_thrusters(tau)
        self._command = np.array([T_port, T_stbd], dtype=np.float32)
        self._last_cmd_vel_time = self.get_clock().now().nanoseconds / 1e9

        # Publish Fossen state
        self._publish_fossen_state()

    def _compute_waypoint_thrust(self):
        """
        Simple P-controller waypoint navigator.
        Computes heading error and distance to current waypoint,
        then produces differential thrust commands [left, right].
        """
        if self._current_wp >= len(self._waypoints):
            self._command = np.array([0.0, 0.0], dtype=np.float32)
            return

        tx, ty = self._waypoints[self._current_wp]
        dx = tx - self._current_x
        dy = ty - self._current_y
        dist = math.hypot(dx, dy)

        if dist < self._wp_reach_tol:
            self.get_logger().info(
                f'Waypoint {self._current_wp} reached ({tx:.1f}, {ty:.1f})'
            )
            self._current_wp += 1
            if self._current_wp >= len(self._waypoints):
                self.get_logger().info('All waypoints completed. Holding position.')
                self._command = np.array([0.0, 0.0], dtype=np.float32)
                return

        tx, ty = self._waypoints[self._current_wp]
        desired_heading = math.atan2(ty - self._current_y, tx - self._current_x)

        heading_err = desired_heading - self._current_yaw
        while heading_err > math.pi:
            heading_err -= 2 * math.pi
        while heading_err < -math.pi:
            heading_err += 2 * math.pi

        thrust = min(self._kp_dist * dist, self._max_thrust)
        turn   = self._kp_heading * heading_err

        left  = np.clip(thrust + turn, -self._max_thrust, self._max_thrust)
        right = np.clip(thrust - turn, -self._max_thrust, self._max_thrust)

        self._command = np.array([left, right], dtype=np.float32)

    def _sim_step(self):
        """Main simulation tick – step HoloOcean and publish sensor data."""

        if self._auto_mode:
            self._compute_waypoint_thrust()

        try:
            state = self.env.step(self._command)
        except Exception as e:
            self.get_logger().warn(f'HoloOcean step failed: {e}')
            return

        if np.any(self._command != self._last_cmd):
            self._last_cmd = self._command.copy()

        now = self.get_clock().now().to_msg()

        # Publish each sensor if present in this state tick
        if 'ProfilingSonar' in state:
            self._publish_sonar(state['ProfilingSonar'], now)

        if 'SidescanSonar' in state:
            self._publish_sidescan(state['SidescanSonar'], now)

        if 'IMUSensor' in state:
            self._publish_imu(state['IMUSensor'], now)

        if 'GPSSensor' in state:
            self._publish_gps(state['GPSSensor'], now)

        if 'DVLSensor' in state:
            self._publish_dvl(state['DVLSensor'], now)

        if 'PoseSensor' in state:
            self._publish_pose(state['PoseSensor'], now)

    # ──────────────────────────────────────────────────────────────────────
    # Sensor publishers
    # ──────────────────────────────────────────────────────────────────────

    def _publish_sonar(self, data: np.ndarray, stamp: Time):
        """
        ProfilingSonar → PointCloud2 + Image

        data shape: (AzimuthBins, RangeBins) float array of intensities
        Points are published in vessel frame (base_link) for relative mapping
        """
        header = Header()
        header.stamp = stamp
        header.frame_id = self.vessel_frame

        # PointCloud2
        pc2 = sonar_to_pointcloud2(
            intensities=data,
            azimuth_deg=self.azimuth_deg,
            range_min=self.range_min,
            range_max=self.range_max,
            elevation_deg=self.elevation_deg,
            intensity_threshold=self.intensity_th,
            header=header
        )
        self.pub_sonar_pc2.publish(pc2)

        # Colour-mapped, resized sonar image for clear obstacle view
        img = profiling_sonar_to_image(
            data, header,
            out_width=self._ps_width,
            out_height=self._ps_height,
            colormap=self._ps_colormap,
            log_scale=self._ps_log_scale,
            dynamic_range_db=self._ps_dyn_range,
        )
        self.pub_sonar_img.publish(img)

    def _publish_sidescan(self, data: np.ndarray, stamp: Time):
        """
        SideScanSonar → sensor_msgs/Image (colour-mapped Waterfall)

        HoloOcean SidescanSonar returns a 1-D intensity array representing
        one ping across the full swath (port + starboard).  The first half
        of the array is the port beam, the second half is starboard.

        Real SSS displays show port reversed so that the nadir (directly
        below the towfish) sits at the centre of the image:
            [port_far … port_near | stbd_near … stbd_far]

        Processing applied:
          • Time-Varying Gain (TVG) — compensates range-dependent attenuation
          • Log-scale compression — 20·log10 to bring out seabed detail
          • Copper / Sonar colourmap — matches HoloOcean or real SSS look
        """
        row = data.flatten().astype(np.float32)

        # ── Arrange as [port_reversed | starboard] ────────────────────
        half = len(row) // 2
        port = row[:half][::-1]   # reverse port beam
        stbd = row[half:]
        ping = np.concatenate([port, stbd])  # nadir at centre

        if self._sidescan_waterfall is None:
            self._sidescan_waterfall = np.zeros(
                (self._waterfall_rows, len(ping)), dtype=np.float32
            )

        # Roll waterfall (shift rows down) and append the new ping at the top
        self._sidescan_waterfall = np.roll(self._sidescan_waterfall, 1, axis=0)
        self._sidescan_waterfall[0, :] = ping

        header = Header()
        header.stamp = stamp
        header.frame_id = self.sonar_frame

        img = sidescan_to_waterfall_image(
            self._sidescan_waterfall,
            header,
            colormap=self._sss_colormap,
            log_scale=self._sss_log_scale,
            tvg_alpha=self._sss_tvg_alpha,
            dynamic_range_db=self._sss_dyn_range,
        )
        self.pub_sidescan_img.publish(img)

    def _publish_imu(self, data: np.ndarray, stamp: Time):
        """
        IMUSensor output shape (with ReturnAccel+ReturnAngVel=true):
          shape (2, 3):  data[0]=[ax,ay,az]  data[1]=[wx,wy,wz]
          shape (6,):    flat [ax,ay,az,wx,wy,wz]
        We flatten to handle both cases safely.
        """
        flat = np.array(data).flatten()

        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.imu_frame

        # Linear acceleration (indices 0-2)
        msg.linear_acceleration.x = float(flat[0])
        msg.linear_acceleration.y = float(flat[1])
        msg.linear_acceleration.z = float(flat[2])

        # Angular velocity (indices 3-5)
        msg.angular_velocity.x = float(flat[3])
        msg.angular_velocity.y = float(flat[4])
        msg.angular_velocity.z = float(flat[5])

        # Orientation unknown from IMU alone — mark covariance as -1
        msg.orientation_covariance[0] = -1.0

        # Covariance (diagonal, approximate)
        accel_var = 0.00277 ** 2
        gyro_var  = 0.00123 ** 2
        msg.linear_acceleration_covariance  = [
            accel_var, 0.0, 0.0,
            0.0, accel_var, 0.0,
            0.0, 0.0, accel_var
        ]
        msg.angular_velocity_covariance = [
            gyro_var, 0.0, 0.0,
            0.0, gyro_var, 0.0,
            0.0, 0.0, gyro_var
        ]

        self.pub_imu.publish(msg)

    def _publish_gps(self, data: np.ndarray, stamp: Time):
        """
        GPSSensor output:
          data[0] → latitude  (degrees)
          data[1] → longitude (degrees)
          data[2] → altitude  (meters)  [if depth=false, this is z offset]
        """
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = self.gps_frame

        msg.status.status  = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude  = float(data[0]) + self.lat_origin
        msg.longitude = float(data[1]) + self.lon_origin
        msg.altitude  = float(data[2])

        # Position covariance: diagonal, sigma ≈ 0.5 m → variance = 0.25
        pos_var = 0.5 ** 2
        msg.position_covariance = [
            pos_var, 0.0, 0.0,
            0.0, pos_var, 0.0,
            0.0, 0.0, pos_var
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub_gps.publish(msg)

    def _publish_dvl(self, data: np.ndarray, stamp: Time):
        """
        DVLSensor output:
          data[0:3] → velocity [vx, vy, vz] in body frame (m/s)
        """
        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.vessel_frame

        msg.twist.linear.x = float(data[0])
        msg.twist.linear.y = float(data[1])
        msg.twist.linear.z = float(data[2])

        self.pub_dvl.publish(msg)

        # ── Sync Fossen body velocities with DVL ground truth ──────────────
        self.fossen.nu[0] = float(data[0])  # surge
        self.fossen.nu[1] = float(data[1])  # sway

    def _publish_pose(self, data: np.ndarray, stamp: Time):
        """
        PoseSensor output:
          data → 4×4 transformation matrix (position + orientation) in world frame

        Publishes:
          - PoseStamped on /holoocean/pose
          - Odometry on /holoocean/odom
          - TF transform world → base_link
        """
        # Extract position and rotation from 4x4 matrix
        T = data  # shape (4,4)
        pos = T[0:3, 3]   # [x, y, z]
        R   = T[0:3, 0:3] # rotation matrix

        q = rotation_matrix_to_quaternion(R)

        # PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.world_frame
        pose_msg.pose.position.x  = float(pos[0])
        pose_msg.pose.position.y  = float(pos[1])
        pose_msg.pose.position.z  = float(pos[2])
        pose_msg.pose.orientation = q
        self.pub_pose.publish(pose_msg)

        # Odometry
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = self.world_frame
        odom.child_frame_id  = self.vessel_frame
        odom.pose.pose.position.x  = float(pos[0])
        odom.pose.pose.position.y  = float(pos[1])
        odom.pose.pose.position.z  = float(pos[2])
        odom.pose.pose.orientation = q
        self.pub_odom.publish(odom)

        # TF: world → base_link
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id  = self.world_frame
        tf.child_frame_id   = self.vessel_frame
        tf.transform.translation.x = float(pos[0])
        tf.transform.translation.y = float(pos[1])
        tf.transform.translation.z = float(pos[2])
        tf.transform.rotation = q
        self.tf_broadcaster.sendTransform(tf)

        # Log vessel position periodically
        self._pose_count = getattr(self, '_pose_count', 0) + 1
        if self._pose_count % 100 == 0:
            self.get_logger().info(f'Vessel pos: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] CMD: {self._command}')

        # Update waypoint controller state
        self._current_x = float(pos[0])
        self._current_y = float(pos[1])
        self._current_yaw = math.atan2(R[1, 0], R[0, 0])

        # ── Sync Fossen model position/heading with ground truth ────────────
        # Keeps the Fossen model from drifting relative to the simulator
        self.fossen.eta[0] = float(pos[0])
        self.fossen.eta[1] = float(pos[1])
        self.fossen.eta[2] = self._current_yaw

        # Static-ish TF: base_link → sonar_link
        # (Sonar is mounted below hull, pointing downward)
        tf_sonar = TransformStamped()
        tf_sonar.header.stamp = stamp
        tf_sonar.header.frame_id = self.vessel_frame
        tf_sonar.child_frame_id  = self.sonar_frame
        tf_sonar.transform.translation.x = 0.0
        tf_sonar.transform.translation.y = 0.0
        tf_sonar.transform.translation.z = -0.3   # 30 cm below keel
        # Sonar points downward: rotate 90° around X
        tf_sonar.transform.rotation = euler_to_quaternion(
            math.pi / 2, 0.0, 0.0
        )
        self.tf_broadcaster.sendTransform(tf_sonar)

        # Vessel marker for RViz visualization
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.vessel_frame
        marker.ns = "vessel"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation = q
        marker.scale.x = 1.0  # length
        marker.scale.y = 0.5  # width
        marker.scale.z = 0.2  # height
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.pub_vessel_marker.publish(marker)

        # Forward arrow marker to show heading
        arrow_marker = Marker()
        arrow_marker.header.stamp = stamp
        arrow_marker.header.frame_id = self.vessel_frame
        arrow_marker.ns = "vessel"
        arrow_marker.id = 1
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD
        arrow_marker.pose.position.x = 0.5
        arrow_marker.pose.position.y = 0.0
        arrow_marker.pose.position.z = 0.15
        arrow_marker.pose.orientation = q
        arrow_marker.scale.x = 1.0  # arrow length
        arrow_marker.scale.y = 0.1  # arrow head width
        arrow_marker.scale.z = 0.1  # arrow head height
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 1.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0
        self.pub_vessel_marker.publish(arrow_marker)

    def _publish_fossen_state(self):
        """Publish Fossen model body velocities and applied forces."""
        stamp = self.get_clock().now().to_msg()
        nu  = self.fossen.nu
        tau = self._fossen_tau

        # Body velocities [u, v, r]
        vel_msg = TwistStamped()
        vel_msg.header.stamp    = stamp
        vel_msg.header.frame_id = self.vessel_frame
        vel_msg.twist.linear.x  = float(nu[0])   # surge [m/s]
        vel_msg.twist.linear.y  = float(nu[1])   # sway  [m/s]
        vel_msg.twist.angular.z = float(nu[2])   # yaw rate [rad/s]
        self.pub_fossen_vel.publish(vel_msg)

        # Generalised forces [X, Y, N]
        force_msg = TwistStamped()
        force_msg.header.stamp    = stamp
        force_msg.header.frame_id = self.vessel_frame
        force_msg.twist.linear.x  = float(tau[0])  # surge force [N]
        force_msg.twist.linear.y  = float(tau[1])  # sway force  [N]
        force_msg.twist.angular.z = float(tau[2])  # yaw moment  [N·m]
        self.pub_fossen_force.publish(force_msg)

    def destroy_node(self):
        """Cleanly close HoloOcean on shutdown."""
        self.get_logger().info('Closing HoloOcean environment...')
        try:
            del self.env
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HoloOceanBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()