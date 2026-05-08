#!/usr/bin/env python3
"""
Semantic Hazard Classifier Node

Architecture
────────────
  Camera → image callback (YOLO thread) → _pending_frame + capture timestamp
  YOLO thread (throttled): decode → resize → infer → publish alert immediately
  Localization thread: for each detection, look up tf at capture time → place
                       a map-frame marker at the estimated HAZMAT location

Retroactive localization
────────────────────────
  The OAK-D stamps each frame at hardware capture time (msg.header.stamp).
  When YOLO finds a detection, we record that stamp and push a job onto a
  small queue.  A separate localization thread then calls tf2's
  lookup_transform(target_frame='map', source_frame='base_link',
  time=capture_stamp) — asking "where was the robot when this photo was
  taken?"  tf2 buffers ~10 s of history, so as long as YOLO finishes within
  that window (it takes ~160 ms) the lookup always succeeds.

  The result is a persistent MarkerArray on /hazmat_map_markers that
  accumulates every confirmed HAZMAT location in the map frame.  This is
  separate from the transient /hazmat_markers which just shows the current
  frame's detection status.

Topics published
────────────────
  /hazard_alert        std_msgs/String      "HAZMAT" | "CLEAR"  (per inference)
  /hazard_markers      MarkerArray          transient detection cylinder
  /hazmat_map_markers  MarkerArray          persistent map-frame HAZMAT pins
  /exploration/stop    std_msgs/Bool        True on critical hazard
"""

import time
import threading
import traceback
import os
import queue
import math

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time as RosTime

import numpy as np
import cv2

from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

import tf2_ros

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

try:
    from irobot_create_msgs.msg import HazardDetectionVector, SlipStatus
    CREATE3_AVAILABLE = True
except ImportError:
    CREATE3_AVAILABLE = False

HAZMAT_AVAILABLE = False
_hazmat_import_error = None
try:
    from frontier_exploration_mapping.deep_hazmat import DeepHAZMAT
    HAZMAT_AVAILABLE = True
except Exception:
    _hazmat_import_error = traceback.format_exc()


# ── Tunable constants ────────────────────────────────────────────────────────

YOLO_INTERVAL_SEC     = 1.0   # minimum gap between YOLO runs
WORKER_IDLE_SLEEP_SEC = 0.1   # sleep when nothing to do
JPEG_QUALITY          = 75    # raw→compressed conversion quality
TF_BUFFER_SEC         = 10.0  # how long tf2 keeps history (default is fine)
TF_TIMEOUT_SEC        = 0.5   # how long to wait for a tf lookup
LOC_QUEUE_MAX         = 20    # max pending localization jobs


class SemanticHazardClassifier(Node):

    def __init__(self):
        super().__init__('semantic_hazard_classifier')

        self.declare_parameter('hazmat_confidence',  0.5)
        self.declare_parameter('yolo_width',         320)
        self.declare_parameter('yolo_interval_sec',  YOLO_INTERVAL_SEC)

        hazmat_confidence   = self.get_parameter('hazmat_confidence').value
        self._yolo_width    = self.get_parameter('yolo_width').value
        self._yolo_interval = self.get_parameter('yolo_interval_sec').value

        self.bridge = CvBridge()

        # ── Shared state ─────────────────────────────────────────────────
        self._pending_frame:    bytes | None = None
        self._pending_stamp:    object       = None   # ROS stamp from msg.header

        self._result_lock     = threading.Lock()
        self._last_detections = []
        self._last_yolo_time  = 0.0

        # ── tf2 buffer — receives /tf and /tf_static on the main executor ─
        self._tf_buffer   = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=TF_BUFFER_SEC))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Localization job queue ────────────────────────────────────────
        # Each item: {'stamp': ROS stamp, 'detections': [...], 'frame_id': str}
        self._loc_queue = queue.Queue(maxsize=LOC_QUEUE_MAX)

        # Accumulated map-frame HAZMAT markers
        self._map_markers: list[Marker] = []
        self._map_marker_lock = threading.Lock()
        self._next_map_marker_id = 0

        # ── DeepHAZMAT ────────────────────────────────────────────────────
        self._hazmat = None
        if HAZMAT_AVAILABLE:
            try:
                pkg_share = get_package_share_directory('frontier_exploration_mapping')
                net_dir   = os.path.join(pkg_share, 'net')
                for fname in ('yolo.cfg', 'yolo.weights', 'labels.names'):
                    fpath = os.path.join(net_dir, fname)
                    if not os.path.exists(fpath):
                        raise FileNotFoundError(f'Missing YOLO file: {fpath}')
                self._hazmat = DeepHAZMAT(
                    k=0,
                    net_directory=net_dir,
                    min_confidence=hazmat_confidence,
                    nms_threshold=0.3,
                    segmentation_enabled=False,
                )
                self.get_logger().info(
                    f'DeepHAZMAT loaded '
                    f'(yolo_width={self._yolo_width}, '
                    f'interval={self._yolo_interval:.1f}s).')
            except Exception as e:
                self.get_logger().error(
                    f'DeepHAZMAT failed to load: {e}\n{traceback.format_exc()}')
        else:
            self.get_logger().error(
                f'DeepHAZMAT import failed:\n{_hazmat_import_error}')

        # ── Image helper node (YOLO thread only) ─────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self._image_node = rclpy.create_node('hazmat_image_helper')
        self._image_node.create_subscription(
            CompressedImage,
            '/oakd/rgb/preview/image_raw/compressed',
            self._compressed_cb,
            sensor_qos,
        )
        self._image_node.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self._raw_cb,
            sensor_qos,
        )
        self._image_executor = rclpy.executors.SingleThreadedExecutor()
        self._image_executor.add_node(self._image_node)

        # ── Background threads ────────────────────────────────────────────
        self._yolo_thread = threading.Thread(
            target=self._yolo_worker, daemon=True, name='yolo_worker')
        self._loc_thread  = threading.Thread(
            target=self._localization_worker, daemon=True, name='loc_worker')
        self._yolo_thread.start()
        self._loc_thread.start()

        # ── Main-node subscribers ─────────────────────────────────────────
        if CREATE3_AVAILABLE:
            self.create_subscription(
                HazardDetectionVector, '/hazard_detection',
                self._hw_hazard_cb, 10)
            self.create_subscription(
                SlipStatus, '/slip_status',
                self._slip_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────
        self.alert_pub      = self.create_publisher(String,      '/hazard_alert',       10)
        self.stop_pub       = self.create_publisher(Bool,        '/exploration/stop',    10)
        self.marker_pub     = self.create_publisher(MarkerArray, '/hazard_markers',      10)
        self.map_marker_pub = self.create_publisher(MarkerArray, '/hazmat_map_markers',  10)

        # 5 Hz heartbeat — keeps RViz2 markers alive between YOLO cycles
        self.create_timer(0.2, self._heartbeat)

        self.get_logger().info(
            f'Semantic Hazard Classifier started.  '
            f'interval={self._yolo_interval:.1f}s  width={self._yolo_width}px\n'
            f'  Retroactive localization: ENABLED  '
            f'(tf buffer={TF_BUFFER_SEC:.0f}s)')

    # ── Image callbacks (YOLO thread) ──────────────────────────────────────

    def _compressed_cb(self, msg: CompressedImage):
        self._pending_frame = bytes(msg.data)
        self._pending_stamp = msg.header.stamp      # hardware capture time

    def _raw_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            ok, buf = cv2.imencode(
                '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if ok:
                self._pending_frame = buf.tobytes()
                self._pending_stamp = msg.header.stamp
        except Exception as e:
            print(f'[image helper] raw decode error: {e}')

    def _hw_hazard_cb(self, msg): pass
    def _slip_cb(self,     msg): pass

    # ── YOLO worker ────────────────────────────────────────────────────────

    def _yolo_worker(self):
        try:
            from imutils import resize as imutils_resize
        except ImportError:
            self.get_logger().error('[YOLO] imutils not installed — thread exiting.')
            return

        self.get_logger().info('[YOLO] worker started.')

        while rclpy.ok():
            # Drain image subscriber queue (non-blocking)
            self._image_executor.spin_once(timeout_sec=0.0)

            # Cooldown
            if time.monotonic() - self._last_yolo_time < self._yolo_interval:
                time.sleep(WORKER_IDLE_SLEEP_SEC)
                continue

            # Grab latest frame + its hardware timestamp
            data  = self._pending_frame
            stamp = self._pending_stamp
            if data is None or self._hazmat is None:
                time.sleep(WORKER_IDLE_SLEEP_SEC)
                continue
            self._pending_frame = None

            # Clear stale result immediately
            with self._result_lock:
                self._last_detections = []

            # Decode → resize
            arr   = np.frombuffer(data, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                time.sleep(WORKER_IDLE_SLEEP_SEC)
                continue
            small = imutils_resize(frame, width=self._yolo_width)

            # Inference
            try:
                results    = self._hazmat.update(small)
                detections = list(results) if results else []
            except Exception as e:
                detections = []
                self.get_logger().error(f'[YOLO] inference error: {e}')

            # Store + publish immediately
            with self._result_lock:
                self._last_detections = detections
            self._publish_current(detections)

            if detections:
                self.get_logger().info(
                    f'[YOLO] {len(detections)} detection(s) — '
                    f'queuing retroactive localization')
                # Push localization job (non-blocking — drop if queue full)
                try:
                    self._loc_queue.put_nowait({
                        'stamp':      stamp,
                        'detections': detections,
                        'frame_id':   'base_link',
                    })
                except queue.Full:
                    self.get_logger().warn('[YOLO] localization queue full — dropping job')

            self._last_yolo_time = time.monotonic()
            time.sleep(WORKER_IDLE_SLEEP_SEC)

    # ── Localization worker ────────────────────────────────────────────────
    #
    # Pops detection jobs off the queue, looks up the robot's map-frame pose
    # at the hardware capture timestamp, then projects a fixed distance
    # forward along the robot's heading to estimate where the HAZMAT is on
    # the wall directly in front of the camera.
    #
    # Proof-of-concept approximation
    # ───────────────────────────────
    #   marker_x = robot_x + PROJECTION_DIST * cos(yaw)
    #   marker_y = robot_y + PROJECTION_DIST * sin(yaw)
    #
    # yaw is extracted from the quaternion returned by tf2.  The projection
    # distance is tunable via HAZMAT_PROJECTION_DIST_M.  Future refinement
    # can replace this with a proper ray-cast using the detection's bounding
    # box centre and the camera's known FOV.

    # How far in front of the robot to place the wall marker (metres).
    # At 0.2 m/s the robot is never closer than ~0.3 m to a wall before
    # Nav2 stops it, so 1.0 m is a reasonable mid-field guess.
    HAZMAT_PROJECTION_DIST_M = 1.0

    def _localization_worker(self):
        self.get_logger().info('[LOC] worker started.')

        while rclpy.ok():
            try:
                job = self._loc_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            stamp      = job['stamp']
            detections = job['detections']

            if stamp is None:
                self.get_logger().warn('[LOC] job has no timestamp — skipping')
                continue

            # Convert ROS stamp to rclpy Time for tf2
            ros_time = RosTime(
                seconds=stamp.sec,
                nanoseconds=stamp.nanosec,
                clock_type=self.get_clock().clock_type,
            )

            try:
                transform = self._tf_buffer.lookup_transform(
                    target_frame='map',
                    source_frame='base_link',
                    time=ros_time,
                    timeout=rclpy.duration.Duration(seconds=TF_TIMEOUT_SEC),
                )
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(f'[LOC] tf lookup failed: {e}')
                continue

            # Robot position in map
            rx = transform.transform.translation.x
            ry = transform.transform.translation.y

            # Extract yaw from quaternion (z-axis rotation only — 2D map)
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            yaw = 2.0 * math.atan2(qz, qw)   # standard 2D yaw extraction

            # Project forward along the robot's heading
            d  = self.HAZMAT_PROJECTION_DIST_M
            mx = rx + d * math.cos(yaw)
            my = ry + d * math.sin(yaw)

            self.get_logger().info(
                f'[LOC] robot ({rx:.2f}, {ry:.2f}) yaw={math.degrees(yaw):.1f}° '
                f'→ HAZMAT projected to ({mx:.2f}, {my:.2f})  '
                f'classes: {[det.name for det in detections]}')

            # Build persistent markers — one cylinder + one text label per detection
            now_stamp = self.get_clock().now().to_msg()
            with self._map_marker_lock:
                for det in detections:
                    # ── Cylinder marker ──────────────────────────────────
                    cyl                    = Marker()
                    cyl.header.frame_id    = 'map'
                    cyl.header.stamp       = now_stamp
                    cyl.ns                 = 'hazmat_locations'
                    cyl.id                 = self._next_map_marker_id
                    self._next_map_marker_id += 1
                    cyl.type               = Marker.CYLINDER
                    cyl.action             = Marker.ADD
                    cyl.pose.position.x    = mx
                    cyl.pose.position.y    = my
                    cyl.pose.position.z    = 0.5
                    cyl.pose.orientation.w = 1.0
                    cyl.scale.x            = 0.4
                    cyl.scale.y            = 0.4
                    cyl.scale.z            = 1.0
                    cyl.color.r            = 1.0
                    cyl.color.g            = 0.5
                    cyl.color.b            = 0.0
                    cyl.color.a            = 0.9
                    cyl.lifetime.sec       = 0   # persistent
                    self._map_markers.append(cyl)

                    # ── Text label above the cylinder ────────────────────
                    txt                    = Marker()
                    txt.header.frame_id    = 'map'
                    txt.header.stamp       = now_stamp
                    txt.ns                 = 'hazmat_labels'
                    txt.id                 = self._next_map_marker_id
                    self._next_map_marker_id += 1
                    txt.type               = Marker.TEXT_VIEW_FACING
                    txt.action             = Marker.ADD
                    txt.pose.position.x    = mx
                    txt.pose.position.y    = my
                    txt.pose.position.z    = 1.2   # above the cylinder
                    txt.pose.orientation.w = 1.0
                    txt.scale.z            = 0.25  # text height in metres
                    txt.color.r            = 1.0
                    txt.color.g            = 1.0
                    txt.color.b            = 1.0
                    txt.color.a            = 1.0
                    txt.text               = det.name.upper()
                    txt.lifetime.sec       = 0
                    self._map_markers.append(txt)

                ma = MarkerArray()
                ma.markers = list(self._map_markers)

            self.map_marker_pub.publish(ma)

    # ── Transient detection publish ────────────────────────────────────────

    def _publish_current(self, detections):
        """Publish the current frame's detection state immediately."""
        stamp = self.get_clock().now().to_msg()
        ma    = MarkerArray()

        if detections:
            for det in detections:
                self.get_logger().info(
                    f'HAZMAT detected: {det}',
                    throttle_duration_sec=1.0)

            alert      = String()
            alert.data = 'HAZMAT'
            self.alert_pub.publish(alert)

            m                    = Marker()
            m.header.frame_id    = 'base_link'
            m.header.stamp       = stamp
            m.ns                 = 'hazmat_current'
            m.id                 = 0
            m.type               = Marker.CYLINDER
            m.action             = Marker.ADD
            m.pose.position.z    = 0.5
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.3
            m.scale.z            = 1.0
            m.color.r            = 1.0
            m.color.g            = 0.0
            m.color.b            = 1.0
            m.color.a            = 0.85
            m.lifetime.sec       = 2
            ma.markers.append(m)

        else:
            alert      = String()
            alert.data = 'CLEAR'
            self.alert_pub.publish(alert)

            clear        = Marker()
            clear.ns     = 'hazmat_current'
            clear.action = Marker.DELETEALL
            ma.markers.append(clear)

        self.marker_pub.publish(ma)

    # ── Heartbeat timer (main executor) ───────────────────────────────────

    def _heartbeat(self):
        """Re-publish current state and accumulated map markers to keep RViz2 alive."""
        with self._result_lock:
            detections = list(self._last_detections)
        self._publish_current(detections)

        # Re-publish persistent map markers so RViz2 doesn't time them out
        with self._map_marker_lock:
            if self._map_markers:
                ma = MarkerArray()
                ma.markers = list(self._map_markers)
                self.map_marker_pub.publish(ma)

    # ── Cleanup ───────────────────────────────────────────────────────────

    def destroy_node(self):
        self._image_executor.shutdown()
        self._image_node.destroy_node()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SemanticHazardClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()