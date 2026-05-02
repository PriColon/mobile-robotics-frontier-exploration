#!/usr/bin/env python3
"""
Semantic Hazard Classifier Node
================================
Detects and classifies environmental hazards using multiple sensor layers:
  Layer 1 - Create3 hardware hazards (cliff, bump, slip)
  Layer 2 - RGB camera (fire, water, smoke, debris, dark zones)
  Layer 3 - Depth camera (potholes, glass walls, unstable terrain)
  Layer 4 - LiDAR (narrow passages, dead ends)
  Layer 5 - DeepHAZMAT YOLO-based HAZMAT sign detection

Publishes:
  /semantic_map       - OccupancyGrid with hazard labels
  /hazard_alert       - Current hazard type as String
  /hazard_markers     - MarkerArray for RViz2 visualization (ALL layers)
  /exploration/stop   - Bool (True if critical hazard detected)

Fix log:
  - Markers now published for ALL detection layers, not just RGB
  - Markers use odom frame so they persist on the map
  - HAZMAT detections generate distinct magenta markers
  - Hardware hazards (cliff/bump) generate markers at robot position
  - LiDAR hazards generate markers in front of robot
  - Text labels added above each hazard marker
  - Added DELETE_ALL on startup to clear stale markers

Authors: Frontier Exploration Team - ASU RAS 598 Spring 2026
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import cv2
import os
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from irobot_create_msgs.msg import HazardDetectionVector, SlipStatus

try:
    from frontier_exploration_mapping.deep_hazmat import DeepHAZMAT
    HAZMAT_AVAILABLE = True
except ImportError:
    HAZMAT_AVAILABLE = False

# ---------------------------------------------------------------------------
# Hazard label constants
# ---------------------------------------------------------------------------
LABEL_CLEAR     = 0
LABEL_DEBRIS    = 30
LABEL_SMOKE     = 50
LABEL_WATER     = 70
LABEL_DARK      = 75
LABEL_NARROW    = 80
LABEL_POTHOLE   = 90
LABEL_GLASS     = 90
LABEL_HAZMAT    = 95
LABEL_FIRE      = 100
LABEL_CLIFF     = 100

LABEL_COLORS = {
    LABEL_CLEAR:   (0.2, 0.8, 0.2, 0.5),
    LABEL_DEBRIS:  (0.9, 0.7, 0.1, 0.8),
    LABEL_SMOKE:   (0.5, 0.5, 0.5, 0.8),
    LABEL_WATER:   (0.1, 0.4, 0.9, 0.9),
    LABEL_DARK:    (0.3, 0.0, 0.5, 0.8),
    LABEL_NARROW:  (1.0, 0.5, 0.0, 0.9),
    LABEL_POTHOLE: (0.6, 0.2, 0.0, 0.9),
    LABEL_GLASS:   (0.7, 0.9, 1.0, 0.8),
    LABEL_HAZMAT:  (1.0, 0.0, 1.0, 1.0),
    LABEL_FIRE:    (1.0, 0.1, 0.0, 1.0),
}

HAZARD_PRIORITY = {
    'CLIFF':    100,
    'FIRE':     100,
    'POTHOLE':  90,
    'GLASS':    85,
    'HAZMAT':   85,
    'WATER':    80,
    'DEAD_END': 80,
    'NARROW':   70,
    'SMOKE':    60,
    'DARK':     60,
    'DEBRIS':   40,
    'CLEAR':    0,
}

CRITICAL_HAZARDS = {'CLIFF', 'FIRE', 'POTHOLE', 'HAZMAT'}

# Per-class HAZMAT danger ratings based on DeepHAZMAT labels.names
HAZMAT_CLASS_LABELS = {
    'explosive':                 100,
    'radioactive':               100,
    'infectious-substance':      100,
    'inhalation-hazard':          95,
    'poison':                     95,
    'flammable':                  90,
    'flammable-solid':            90,
    'spontaneously-combustible':  90,
    'organic-peroxide':           85,
    'oxidizer':                   85,
    'corrosive':                  80,
    'non-flammable-gas':          75,
    'oxygen':                     75,
    'dangerous':                  70,
}

# HAZMAT classes that trigger E-stop
CRITICAL_HAZMAT_CLASSES = {
    'explosive', 'radioactive',
    'infectious-substance', 'inhalation-hazard', 'poison'
}


# ---------------------------------------------------------------------------
# Layer 2: RGB Visual Hazard Classifier
# ---------------------------------------------------------------------------

def classify_patch(patch_bgr):
    if patch_bgr is None or patch_bgr.size == 0:
        return 'CLEAR', LABEL_CLEAR

    hsv  = cv2.cvtColor(patch_bgr, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(patch_bgr, cv2.COLOR_BGR2GRAY)
    H = hsv[:, :, 0].astype(float)
    S = hsv[:, :, 1].astype(float)
    V = hsv[:, :, 2].astype(float)
    total = H.size

    fire_mask = (((H < 20) | (H > 160)) & (S > 120) & (V > 120))
    if np.sum(fire_mask) / total > 0.12:
        return 'FIRE', LABEL_FIRE

    water_mask = (H > 90) & (H < 130) & (S > 20) & (V > 80)
    shiny_mask = (S < 30) & (V > 180)
    if np.sum(water_mask) / total > 0.18 or np.sum(shiny_mask) / total > 0.25:
        return 'WATER', LABEL_WATER

    smoke_mask = (S < 30) & (V > 60) & (V < 170)
    if np.sum(smoke_mask) / total > 0.55:
        return 'SMOKE', LABEL_SMOKE

    hazmat_yellow = (H > 18) & (H < 36) & (S > 100) & (V > 100)
    if np.sum(hazmat_yellow) / total > 0.15:
        return 'HAZMAT', LABEL_HAZMAT

    edges = cv2.Canny(gray, 50, 150)
    if np.sum(edges > 0) / total > 0.22:
        return 'DEBRIS', LABEL_DEBRIS

    if V.mean() < 45:
        return 'DARK', LABEL_DARK

    return 'CLEAR', LABEL_CLEAR


def classify_frame(frame_bgr, rows=3, cols=4):
    if frame_bgr is None:
        return []
    h, w = frame_bgr.shape[:2]
    ph, pw = h // rows, w // cols
    results = []
    for r in range(rows):
        for c in range(cols):
            patch = frame_bgr[r * ph:(r + 1) * ph, c * pw:(c + 1) * pw]
            name, value = classify_patch(patch)
            results.append((r, c, name, value))
    return results


# ---------------------------------------------------------------------------
# Layer 3: Depth Hazard Analysis
# ---------------------------------------------------------------------------

def classify_depth(depth_frame):
    if depth_frame is None or depth_frame.size == 0:
        return 'CLEAR', LABEL_CLEAR

    h, w = depth_frame.shape
    valid = depth_frame[np.isfinite(depth_frame) & (depth_frame > 0)]

    if valid.size == 0:
        return 'GLASS', LABEL_GLASS

    nan_ratio = 1.0 - (valid.size / depth_frame.size)
    if nan_ratio > 0.50:
        return 'GLASS', LABEL_GLASS

    floor = depth_frame[2 * h // 3:, w // 4: 3 * w // 4]
    floor_valid = floor[np.isfinite(floor) & (floor > 0)]
    if floor_valid.size > 0 and floor_valid.mean() > 1.5:
        return 'POTHOLE', LABEL_POTHOLE

    centre = depth_frame[h // 4: 3 * h // 4, w // 4: 3 * w // 4]
    centre_valid = centre[np.isfinite(centre) & (centre > 0)]
    if centre_valid.size > 0 and centre_valid.var() > 0.35:
        return 'DEBRIS', LABEL_DEBRIS

    return 'CLEAR', LABEL_CLEAR


# ---------------------------------------------------------------------------
# Layer 4: LiDAR Hazard Analysis
# ---------------------------------------------------------------------------

def classify_lidar(ranges, angle_min, angle_increment):
    if not ranges:
        return 'CLEAR', LABEL_CLEAR

    n = len(ranges)
    arr = np.array(ranges, dtype=float)
    arr[~np.isfinite(arr)] = 0.0

    def sector_min(deg_start, deg_end):
        i0 = int((np.radians(deg_start) - angle_min) / angle_increment) % n
        i1 = int((np.radians(deg_end) - angle_min) / angle_increment) % n
        indices = (list(range(i0, i1)) if i0 < i1
                   else list(range(i0, n)) + list(range(0, i1)))
        vals = arr[indices]
        vals = vals[vals > 0]
        return vals.min() if vals.size > 0 else 999.0

    front_min = sector_min(-30, 30)
    left_min  = sector_min(60, 120)
    right_min = sector_min(-120, -60)

    if front_min < 0.40 and left_min < 0.60 and right_min < 0.60:
        return 'DEAD_END', LABEL_NARROW
    if left_min < 0.35 and right_min < 0.35:
        return 'NARROW', LABEL_NARROW
    if front_min < 0.25:
        return 'DEBRIS', LABEL_DEBRIS

    return 'CLEAR', LABEL_CLEAR


# ---------------------------------------------------------------------------
# Hazard Fusion
# ---------------------------------------------------------------------------

def fuse_hazards(hazard_list):
    if not hazard_list:
        return 'CLEAR', LABEL_CLEAR
    return max(hazard_list, key=lambda x: HAZARD_PRIORITY.get(x[0], 0))


# ---------------------------------------------------------------------------
# Marker helpers
# ---------------------------------------------------------------------------

def make_cube_marker(ns, marker_id, x, y, z, label_value, stamp,
                     frame_id='odom', scale=0.3, lifetime_sec=2):
    m = Marker()
    m.header.frame_id    = frame_id
    m.header.stamp       = stamp
    m.ns                 = ns
    m.id                 = marker_id
    m.type               = Marker.CUBE
    m.action             = Marker.ADD
    m.pose.position.x    = x
    m.pose.position.y    = y
    m.pose.position.z    = z
    m.pose.orientation.w = 1.0
    m.scale.x = scale
    m.scale.y = scale
    m.scale.z = scale * 0.4
    col = LABEL_COLORS.get(label_value, (0.5, 0.5, 0.5, 0.8))
    m.color.r = col[0]
    m.color.g = col[1]
    m.color.b = col[2]
    m.color.a = col[3]
    m.lifetime.sec     = lifetime_sec
    m.lifetime.nanosec = 0
    return m


def make_text_marker(ns, marker_id, x, y, z, text, stamp,
                     frame_id='odom', lifetime_sec=2):
    m = Marker()
    m.header.frame_id    = frame_id
    m.header.stamp       = stamp
    m.ns                 = ns + '_label'
    m.id                 = marker_id
    m.type               = Marker.TEXT_VIEW_FACING
    m.action             = Marker.ADD
    m.pose.position.x    = x
    m.pose.position.y    = y
    m.pose.position.z    = z + 0.3
    m.pose.orientation.w = 1.0
    m.scale.z  = 0.14
    m.color.r  = 1.0
    m.color.g  = 1.0
    m.color.b  = 1.0
    m.color.a  = 1.0
    m.text     = text
    m.lifetime.sec     = lifetime_sec
    m.lifetime.nanosec = 0
    return m


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------

class SemanticHazardClassifier(Node):

    def __init__(self):
        super().__init__('semantic_hazard_classifier')

        self.declare_parameter('grid_rows',         3)
        self.declare_parameter('grid_cols',         4)
        self.declare_parameter('map_resolution',    0.5)
        self.declare_parameter('hazmat_confidence', 0.5)

        self.grid_rows      = self.get_parameter('grid_rows').value
        self.grid_cols      = self.get_parameter('grid_cols').value
        self.map_resolution = self.get_parameter('map_resolution').value
        hazmat_confidence   = self.get_parameter('hazmat_confidence').value

        self.bridge     = CvBridge()
        self._latest_rgb   = None
        self._latest_depth = None
        self._latest_scan  = None
        self._hw_hazards   = []
        self._slip_active  = False
        self._robot_x      = 0.0
        self._robot_y      = 0.0
        self._marker_id    = 0

        # Layer 5: DeepHAZMAT
        self._hazmat = None
        if HAZMAT_AVAILABLE:
            try:
                pkg_share = get_package_share_directory(
                    'frontier_exploration_mapping')
                net_dir = os.path.join(pkg_share, 'net')
                self._hazmat = DeepHAZMAT(
                    k=0,
                    net_directory=net_dir,
                    min_confidence=hazmat_confidence,
                    nms_threshold=0.3,
                    segmentation_enabled=False,
                )
                self.get_logger().info('DeepHAZMAT loaded successfully.')
            except Exception as e:
                self.get_logger().warn(f'DeepHAZMAT failed to load: {e}')
        else:
            self.get_logger().warn('deep_hazmat not available.')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Subscribers
        self.create_subscription(Image, '/oakd/rgb/preview/image_raw',
                                 self._rgb_callback, sensor_qos)
        self.create_subscription(Image, '/oakd/rgb/preview/depth',
                                 self._depth_callback, sensor_qos)
        self.create_subscription(LaserScan, '/scan',
                                 self._scan_callback, sensor_qos)
        self.create_subscription(HazardDetectionVector, '/hazard_detection',
                                 self._hazard_hw_callback, 10)
        self.create_subscription(SlipStatus, '/slip_status',
                                 self._slip_callback, 10)
        self.create_subscription(Odometry, '/odom',
                                 self._odom_callback, sensor_qos)

        # Publishers
        self.semantic_map_pub = self.create_publisher(
            OccupancyGrid, '/semantic_map', 10)
        self.alert_pub  = self.create_publisher(
            String,      '/hazard_alert',    10)
        self.stop_pub   = self.create_publisher(
            Bool,        '/exploration/stop', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/hazard_markers',   10)

        # Clear stale markers after 1 second
        self._clear_timer = self.create_timer(1.0, self._clear_stale_markers_once)

        # Main processing at 10 Hz
        self.create_timer(0.1, self._process)

        self.get_logger().info('Semantic Hazard Classifier started.')

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _rgb_callback(self, msg):
        try:
            self._latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'RGB error: {e}')

    def _depth_callback(self, msg):
        try:
            self._latest_depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        except Exception as e:
            self.get_logger().warn(f'Depth error: {e}')

    def _scan_callback(self, msg):
        self._latest_scan = msg

    def _hazard_hw_callback(self, msg):
        self._hw_hazards = [h.type for h in msg.detections]

    def _slip_callback(self, msg):
        self._slip_active = msg.is_slipping

    def _odom_callback(self, msg):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

    def _clear_stale_markers_once(self):
        """Run once after startup to clear any leftover RViz2 markers."""
        ma = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        ma.markers.append(m)
        self.marker_pub.publish(ma)
        self.get_logger().info('Cleared stale markers.')
        self._clear_timer.cancel()

    # ── Main Processing ─────────────────────────────────────────────────────

    def _process(self):
        stamp = self.get_clock().now().to_msg()
        hazards_detected = []
        markers = []
        rx, ry = self._robot_x, self._robot_y

        # ── Layer 1: Hardware ──────────────────────────────────────────────
        if 2 in self._hw_hazards:
            hazards_detected.append(('CLIFF', LABEL_CLIFF))
            mid = self._next_id()
            markers.append(make_cube_marker(
                'hw', mid, rx, ry, 0.1, LABEL_CLIFF, stamp,
                scale=0.4, lifetime_sec=3))
            markers.append(make_text_marker(
                'hw', mid, rx, ry, 0.1, 'CLIFF', stamp, lifetime_sec=3))

        if 1 in self._hw_hazards:
            hazards_detected.append(('DEBRIS', LABEL_DEBRIS))
            mid = self._next_id()
            markers.append(make_cube_marker(
                'hw', mid, rx, ry, 0.1, LABEL_DEBRIS, stamp, lifetime_sec=2))
            markers.append(make_text_marker(
                'hw', mid, rx, ry, 0.1, 'BUMP', stamp, lifetime_sec=2))

        if self._slip_active:
            hazards_detected.append(('WATER', LABEL_WATER))
            mid = self._next_id()
            markers.append(make_cube_marker(
                'hw', mid, rx, ry, 0.1, LABEL_WATER, stamp, lifetime_sec=2))
            markers.append(make_text_marker(
                'hw', mid, rx, ry, 0.1, 'SLIP', stamp, lifetime_sec=2))

        # ── Layer 2: RGB camera ────────────────────────────────────────────
        rgb_results = []
        if self._latest_rgb is not None:
            rgb_results = classify_frame(
                self._latest_rgb, self.grid_rows, self.grid_cols)
            for (r, c, name, value) in rgb_results:
                if name == 'CLEAR':
                    continue
                hazards_detected.append((name, value))
                # Project patch position in front of robot
                ox = 0.4 + (self.grid_rows - r) * self.map_resolution
                oy = (c - self.grid_cols / 2.0) * self.map_resolution
                mid = self._next_id()
                markers.append(make_cube_marker(
                    'rgb', mid, rx + ox, ry + oy, 0.05, value, stamp,
                    scale=self.map_resolution * 0.85, lifetime_sec=2))
                markers.append(make_text_marker(
                    'rgb', mid, rx + ox, ry + oy, 0.05, name, stamp))

        # ── Layer 3: Depth ─────────────────────────────────────────────────
        if self._latest_depth is not None:
            d_name, d_val = classify_depth(self._latest_depth)
            if d_name != 'CLEAR':
                hazards_detected.append((d_name, d_val))
                mid = self._next_id()
                markers.append(make_cube_marker(
                    'depth', mid, rx + 0.6, ry, 0.05, d_val, stamp,
                    scale=0.35, lifetime_sec=2))
                markers.append(make_text_marker(
                    'depth', mid, rx + 0.6, ry, 0.05,
                    f'DEPTH:{d_name}', stamp))

        # ── Layer 4: LiDAR ─────────────────────────────────────────────────
        if self._latest_scan is not None:
            s = self._latest_scan
            l_name, l_val = classify_lidar(
                list(s.ranges), s.angle_min, s.angle_increment)
            if l_name != 'CLEAR':
                hazards_detected.append((l_name, l_val))
                mid = self._next_id()
                markers.append(make_cube_marker(
                    'lidar', mid, rx + 0.5, ry, 0.1, l_val, stamp,
                    scale=0.4, lifetime_sec=2))
                markers.append(make_text_marker(
                    'lidar', mid, rx + 0.5, ry, 0.1,
                    f'LIDAR:{l_name}', stamp))

        # ── Layer 5: DeepHAZMAT ────────────────────────────────────────────
        if self._hazmat is not None and self._latest_rgb is not None:
            try:
                from imutils import resize
                frame = resize(self._latest_rgb.copy(), width=640)
                detections = list(self._hazmat.update(frame))
                if detections:
                    for det in detections:
                        class_name = det.name.lower().strip()
                        label_val = HAZMAT_CLASS_LABELS.get(
                            class_name, LABEL_HAZMAT)
                        hazards_detected.append(('HAZMAT', label_val))
                        mid = self._next_id()
                        markers.append(make_cube_marker(
                            'hazmat', mid,
                            rx + 0.8, ry, 0.2,
                            label_val, stamp,
                            scale=0.5, lifetime_sec=5))
                        markers.append(make_text_marker(
                            'hazmat', mid,
                            rx + 0.8, ry, 0.2,
                            f'{class_name.upper()} {det.confidence_string()}',
                            stamp, lifetime_sec=5))
                        # E-stop for critical HAZMAT classes
                        if class_name in CRITICAL_HAZMAT_CLASSES:
                            stop = Bool()
                            stop.data = True
                            self.stop_pub.publish(stop)
                            self.get_logger().warn(
                                f'CRITICAL HAZMAT: {class_name} '
                                f'({det.confidence_string()}) — E-STOP')
                        else:
                            self.get_logger().info(
                                f'HAZMAT: {class_name} '
                                f'({det.confidence_string()}) '
                                f'label={label_val}')
            except Exception as e:
                self.get_logger().warn(f'HAZMAT error: {e}')

        # ── Fuse ───────────────────────────────────────────────────────────
        top_name, top_value = fuse_hazards(hazards_detected)

        # ── Publish markers ────────────────────────────────────────────────
        if markers:
            ma = MarkerArray()
            ma.markers = markers
            self.marker_pub.publish(ma)

        # ── Publish semantic map ───────────────────────────────────────────
        self._publish_semantic_map(rgb_results, stamp)

        # ── Publish alert ──────────────────────────────────────────────────
        alert = String()
        alert.data = top_name
        self.alert_pub.publish(alert)

        # ── E-stop if critical ─────────────────────────────────────────────
        if top_name in CRITICAL_HAZARDS:
            stop = Bool()
            stop.data = True
            self.stop_pub.publish(stop)
            self.get_logger().warn(
                f'CRITICAL HAZARD: {top_name} — E-STOP TRIGGERED')

        if top_name != 'CLEAR':
            self.get_logger().info(
                f'Hazard: {top_name} (label={top_value})')

    def _publish_semantic_map(self, rgb_results, stamp):
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = 'odom'
        grid_msg.header.stamp    = stamp
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width      = self.grid_cols
        grid_msg.info.height     = self.grid_rows
        grid_msg.info.origin.position.x = (
            self._robot_x - self.grid_rows * self.map_resolution / 2)
        grid_msg.info.origin.position.y = (
            self._robot_y - self.grid_cols * self.map_resolution / 2)
        grid_msg.info.origin.orientation.w = 1.0

        data = [0] * (self.grid_rows * self.grid_cols)
        for (r, c, name, value) in rgb_results:
            idx = r * self.grid_cols + c
            if 0 <= idx < len(data):
                data[idx] = value
        grid_msg.data = data
        self.semantic_map_pub.publish(grid_msg)

    def _next_id(self):
        mid = self._marker_id
        self._marker_id += 1
        return mid


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = SemanticHazardClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
