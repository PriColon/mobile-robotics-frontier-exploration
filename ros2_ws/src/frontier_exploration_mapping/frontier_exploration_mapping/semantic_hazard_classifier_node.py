#!/usr/bin/env python3
"""
Semantic Hazard Classifier Node
Detects and classifies environmental hazards using multiple sensor layers:
  Layer 1 - Create3 hardware hazards (cliff, bump, slip)
  Layer 2 - RGB camera (fire, water, smoke, debris, dark zones)
  Layer 3 - Depth camera (potholes, glass walls, unstable terrain)
  Layer 4 - LiDAR (narrow passages, dead ends)
  Layer 5 - DeepHAZMAT YOLO-based HAZMAT sign detection (merged)

Publishes:
  /semantic_map       - OccupancyGrid with hazard labels
  /hazard_alert       - Current hazard type as String
  /hazard_markers     - MarkerArray for RViz2 visualization
  /exploration/stop   - Bool (True if critical hazard detected)

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

# ROS2 message types
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

# Create3 / iRobot messages
from irobot_create_msgs.msg import HazardDetectionVector, SlipStatus

# DeepHAZMAT — local library inside this package
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
    LABEL_DEBRIS:  (0.9, 0.7, 0.1, 0.7),
    LABEL_SMOKE:   (0.5, 0.5, 0.5, 0.7),
    LABEL_WATER:   (0.1, 0.4, 0.9, 0.8),
    LABEL_DARK:    (0.3, 0.0, 0.5, 0.7),
    LABEL_NARROW:  (1.0, 0.5, 0.0, 0.8),
    LABEL_POTHOLE: (0.6, 0.2, 0.0, 0.9),
    LABEL_GLASS:   (0.7, 0.9, 1.0, 0.6),
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

    # FIRE — orange/red
    fire_mask = (((H < 20) | (H > 160)) & (S > 120) & (V > 120))
    if np.sum(fire_mask) / total > 0.12:
        return 'FIRE', LABEL_FIRE

    # WATER / WET FLOOR
    water_mask = (H > 90) & (H < 130) & (S > 20) & (V > 80)
    shiny_mask = (S < 30) & (V > 180)
    if np.sum(water_mask) / total > 0.18 or np.sum(shiny_mask) / total > 0.25:
        return 'WATER', LABEL_WATER

    # SMOKE
    smoke_mask = (S < 30) & (V > 60) & (V < 170)
    if np.sum(smoke_mask) / total > 0.55:
        return 'SMOKE', LABEL_SMOKE

    # WARNING TAPE / HAZMAT colour cue
    hazmat_yellow = (H > 18) & (H < 36) & (S > 100) & (V > 100)
    if np.sum(hazmat_yellow) / total > 0.15:
        return 'HAZMAT', LABEL_HAZMAT

    # DEBRIS — high edge density
    edges = cv2.Canny(gray, 50, 150)
    if np.sum(edges > 0) / total > 0.22:
        return 'DEBRIS', LABEL_DEBRIS

    # DARK ZONE
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
# ROS2 Node
# ---------------------------------------------------------------------------

class SemanticHazardClassifier(Node):

    def __init__(self):
        super().__init__('semantic_hazard_classifier')

        self.declare_parameter('grid_rows', 3)
        self.declare_parameter('grid_cols', 4)
        self.declare_parameter('map_resolution', 0.5)
        self.declare_parameter('use_sim_time', False)
        self.declare_parameter('hazmat_confidence', 0.5)

        self.grid_rows      = self.get_parameter('grid_rows').value
        self.grid_cols      = self.get_parameter('grid_cols').value
        self.map_resolution = self.get_parameter('map_resolution').value
        hazmat_confidence   = self.get_parameter('hazmat_confidence').value

        self.bridge = CvBridge()

        # Internal state
        self._latest_rgb   = None
        self._latest_depth = None
        self._latest_scan  = None
        self._hw_hazards   = []
        self._slip_active  = False

        # ── Layer 5: DeepHAZMAT ───────────────────────────────────────────
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
                self._hazmat = None
        else:
            self.get_logger().warn(
                'deep_hazmat not available — HAZMAT detection disabled.')

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Subscribers
        self.create_subscription(
            Image, '/oakd/rgb/preview/image_raw',
            self._rgb_callback, sensor_qos)

        self.create_subscription(
            Image, '/oakd/rgb/preview/depth',
            self._depth_callback, sensor_qos)

        self.create_subscription(
            LaserScan, '/scan',
            self._scan_callback, sensor_qos)

        self.create_subscription(
            HazardDetectionVector, '/hazard_detection',
            self._hazard_hw_callback, 10)

        self.create_subscription(
            SlipStatus, '/slip_status',
            self._slip_callback, 10)

        # Publishers
        self.semantic_map_pub = self.create_publisher(
            OccupancyGrid, '/semantic_map', 10)
        self.alert_pub = self.create_publisher(
            String, '/hazard_alert', 10)
        self.stop_pub = self.create_publisher(
            Bool, '/exploration/stop', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/hazard_markers', 10)

        # Processing timer 10 Hz
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

    # ── Main Processing ─────────────────────────────────────────────────────

    def _process(self):
        hazards_detected = []

        # Layer 1: Hardware
        if 2 in self._hw_hazards:
            hazards_detected.append(('CLIFF', LABEL_CLIFF))
        if 1 in self._hw_hazards:
            hazards_detected.append(('DEBRIS', LABEL_DEBRIS))
        if self._slip_active:
            hazards_detected.append(('WATER', LABEL_WATER))

        # Layer 2: RGB
        rgb_results = []
        if self._latest_rgb is not None:
            rgb_results = classify_frame(
                self._latest_rgb, self.grid_rows, self.grid_cols)
            for (r, c, name, value) in rgb_results:
                if name != 'CLEAR':
                    hazards_detected.append((name, value))

        # Layer 3: Depth
        if self._latest_depth is not None:
            depth_name, depth_val = classify_depth(self._latest_depth)
            if depth_name != 'CLEAR':
                hazards_detected.append((depth_name, depth_val))

        # Layer 4: LiDAR
        if self._latest_scan is not None:
            scan = self._latest_scan
            lidar_name, lidar_val = classify_lidar(
                list(scan.ranges), scan.angle_min, scan.angle_increment)
            if lidar_name != 'CLEAR':
                hazards_detected.append((lidar_name, lidar_val))

        # Layer 5: DeepHAZMAT
        if self._hazmat is not None and self._latest_rgb is not None:
            try:
                from imutils import resize
                frame = resize(self._latest_rgb.copy(), width=640)
                for hazmat in self._hazmat.update(frame):
                    hazards_detected.append(('HAZMAT', LABEL_HAZMAT))
                    self.get_logger().info(
                        f'HAZMAT sign detected: {hazmat}')
            except Exception as e:
                self.get_logger().warn(f'HAZMAT detection error: {e}')

        # Fuse
        top_name, top_value = fuse_hazards(hazards_detected)

        # Publish
        self._publish_semantic_map(rgb_results, top_name, top_value)

        alert = String()
        alert.data = top_name
        self.alert_pub.publish(alert)

        if top_name in CRITICAL_HAZARDS:
            stop = Bool()
            stop.data = True
            self.stop_pub.publish(stop)
            self.get_logger().warn(
                f'CRITICAL HAZARD: {top_name} — E-STOP TRIGGERED')

        if top_name != 'CLEAR':
            self.get_logger().info(
                f'Hazard: {top_name} (value={top_value})')

    def _publish_semantic_map(self, rgb_results, top_name, top_value):
        stamp = self.get_clock().now().to_msg()

        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = 'base_link'
        grid_msg.header.stamp = stamp
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = self.grid_cols
        grid_msg.info.height = self.grid_rows
        grid_msg.info.origin.position.x = -(
            self.grid_rows * self.map_resolution / 2)
        grid_msg.info.origin.position.y = -(
            self.grid_cols * self.map_resolution / 2)
        grid_msg.info.origin.orientation.w = 1.0

        data = [0] * (self.grid_rows * self.grid_cols)
        for (r, c, name, value) in rgb_results:
            idx = r * self.grid_cols + c
            if 0 <= idx < len(data):
                data[idx] = value
        grid_msg.data = data
        self.semantic_map_pub.publish(grid_msg)

        ma = MarkerArray()
        for (r, c, name, value) in rgb_results:
            if value == LABEL_CLEAR:
                continue
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = stamp
            marker.ns = 'semantic_hazards'
            marker.id = r * self.grid_cols + c
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = (
                (self.grid_rows - r - 0.5) * self.map_resolution - 0.3)
            marker.pose.position.y = (
                (c - self.grid_cols / 2 + 0.5) * self.map_resolution)
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.map_resolution * 0.9
            marker.scale.y = self.map_resolution * 0.9
            marker.scale.z = 0.10
            col = LABEL_COLORS.get(value, (0.5, 0.5, 0.5, 0.7))
            marker.color.r = col[0]
            marker.color.g = col[1]
            marker.color.b = col[2]
            marker.color.a = col[3]
            marker.lifetime.sec = 1
            marker.lifetime.nanosec = 0
            ma.markers.append(marker)

        self.marker_pub.publish(ma)


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
