#!/usr/bin/env python3
"""
Semantic Hazard Classifier Node  (YOLO-only, throttled + CPU-friendly)

Key design points:
  - YOLO runs at most once every YOLO_INTERVAL_SEC (default 1.0 s) to stay
    CPU-friendly.  At 0.2 m/s the robot travels only 0.2 m between inferences.
  - After every inference the result is published IMMEDIATELY on the YOLO thread
    via a direct call to _do_publish(), bypassing the 5 Hz timer.  This means
    detections appear (and disappear) within one inference cycle of the scene change
    rather than waiting up to YOLO_INTERVAL_SEC for the timer to fire.
  - _last_detections is CLEARED at the start of each inference cycle so that a
    stale positive can never outlive its own YOLO run.
  - The 5 Hz timer now acts only as a heartbeat keepalive — it re-publishes the
    current (possibly empty) state so RViz2 markers don't time out between inferences.
  - All heavy work stays on the YOLO thread; the main executor handles only
    lightweight timer callbacks and Create3 hardware events.

Data flow:
  camera topic → image callbacks (YOLO thread) → _pending_frame
  YOLO thread: cooldown elapsed → decode → resize → infer → _do_publish() [immediate]
  main executor: 5 Hz heartbeat timer → _do_publish() [keepalive only]
"""

import time
import threading
import traceback
import os

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import cv2

from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

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


# ---------------------------------------------------------------------------
# Tunable constants
# ---------------------------------------------------------------------------

# How often YOLO is allowed to run.
# At 0.2 m/s the robot travels 0.2 m in 1 s — scene barely changes.
# Results are published immediately after each run so lowering this only
# increases CPU load without improving responsiveness.
YOLO_INTERVAL_SEC = 1.0

# How long the YOLO worker sleeps between loop iterations when it has nothing to do.
# 0.1 s = 10 Hz polling — cheap, keeps latency low.
WORKER_IDLE_SLEEP_SEC = 0.1

# JPEG quality for raw→compressed conversion (lower = faster, still fine for YOLO).
JPEG_QUALITY = 75


class SemanticHazardClassifier(Node):

    def __init__(self):
        super().__init__('semantic_hazard_classifier')

        self.declare_parameter('hazmat_confidence', 0.5)
        self.declare_parameter('yolo_width', 320)
        self.declare_parameter('yolo_interval_sec', YOLO_INTERVAL_SEC)

        hazmat_confidence  = self.get_parameter('hazmat_confidence').value
        self._yolo_width   = self.get_parameter('yolo_width').value
        self._yolo_interval = self.get_parameter('yolo_interval_sec').value

        self.bridge = CvBridge()

        # ── Shared state ─────────────────────────────────────────────────
        # _pending_frame: latest raw JPEG bytes from the camera callback.
        #   Written by image callbacks (YOLO thread).
        #   Read+cleared by YOLO inference (YOLO thread).
        #   Both sides are on the SAME thread → no lock needed here.
        self._pending_frame: bytes | None = None

        # _last_detections: result of the last successful inference.
        #   Written by YOLO thread, read by publish timer (main thread) → needs lock.
        self._result_lock     = threading.Lock()
        self._last_detections = []

        # Timestamp of the last YOLO run (monotonic, YOLO thread only).
        self._last_yolo_time = 0.0

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

        # ── Image helper node (lives entirely on the YOLO thread) ─────────
        # This node has NO publishers and is never added to the main executor.
        # The main rclpy.spin() will never touch it.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,          # depth=1: only keep the newest frame, drop the rest
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

        # ── YOLO background thread ────────────────────────────────────────
        self._yolo_thread = threading.Thread(target=self._yolo_worker, daemon=True)
        self._yolo_thread.start()

        # ── Main-node subscribers (no images) ─────────────────────────────
        if CREATE3_AVAILABLE:
            self.create_subscription(
                HazardDetectionVector, '/hazard_detection',
                self._hw_hazard_cb, 10)
            self.create_subscription(
                SlipStatus, '/slip_status',
                self._slip_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────
        self.alert_pub  = self.create_publisher(String,      '/hazard_alert',     10)
        self.stop_pub   = self.create_publisher(Bool,        '/exploration/stop',  10)
        self.marker_pub = self.create_publisher(MarkerArray, '/hazard_markers',    10)

        # 5 Hz heartbeat — re-publishes current state so RViz2 markers don't time out.
        # The real publish happens immediately after each YOLO inference on the YOLO thread.
        self.create_timer(0.2, self._publish_results)

        self.get_logger().info(
            f'Semantic Hazard Classifier started. '
            f'YOLO will run at most every {self._yolo_interval:.1f} s.')

    # ── Image callbacks — called on YOLO thread by _image_executor ────────
    # These only store the latest JPEG bytes; they do NOT decode or run YOLO.
    # Storing raw bytes is a fast memcopy — essentially free.

    def _compressed_cb(self, msg: CompressedImage):
        # Overwrite whatever was pending — we only ever need the newest frame.
        self._pending_frame = bytes(msg.data)

    def _raw_cb(self, msg: Image):
        # Raw Image: encode to JPEG on this thread so downstream handling is uniform.
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            ok, buf = cv2.imencode(
                '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if ok:
                self._pending_frame = buf.tobytes()
        except Exception as e:
            print(f'[image helper] raw decode error: {e}')

    def _hw_hazard_cb(self, msg):
        from irobot_create_msgs.msg import HazardDetection
        for det in msg.detections:
            if det.type in (HazardDetection.CLIFF,):
                stop = Bool()
                stop.data = True
                self.stop_pub.publish(stop)
                self.get_logger().warn(f'CLIFF detected — E-STOP triggered')
                return

    def _slip_cb(self, msg):
        pass   # placeholder for slip detection

    # ── YOLO worker ────────────────────────────────────────────────────────
    #
    # Loop structure:
    #   1. spin_once — drives image callbacks to fill _pending_frame (fast, non-blocking)
    #   2. Check cooldown — if not enough time has passed, sleep and continue
    #   3. Grab frame — if nothing new arrived, sleep and continue
    #   4. CLEAR _last_detections immediately — stale result is gone from this point
    #   5. Decode + resize + infer
    #   6. Store result and publish IMMEDIATELY (don't wait for the 5 Hz timer)
    #   7. Sleep WORKER_IDLE_SLEEP_SEC before next iteration

    def _yolo_worker(self):
        try:
            from imutils import resize as imutils_resize
        except ImportError:
            print('[YOLO worker] imutils not installed — exiting thread.')
            return

        self.get_logger().info('[YOLO worker] thread started.')

        while rclpy.ok():
            # ── Step 1: drain the image subscriber queue (fast) ──────────
            self._image_executor.spin_once(timeout_sec=0.0)

            # ── Step 2: cooldown check ────────────────────────────────────
            now = time.monotonic()
            if now - self._last_yolo_time < self._yolo_interval:
                time.sleep(WORKER_IDLE_SLEEP_SEC)
                continue

            # ── Step 3: grab the latest frame ────────────────────────────
            data = self._pending_frame
            if data is None or self._hazmat is None:
                time.sleep(WORKER_IDLE_SLEEP_SEC)
                continue
            self._pending_frame = None  # consume

            # ── Step 4: CLEAR stale result immediately ────────────────────
            # From this moment forward, _last_detections reflects THIS inference
            # cycle.  Any result from a previous cycle is gone — it will not be
            # re-broadcast by the heartbeat timer while we're inferring.
            with self._result_lock:
                self._last_detections = []

            # ── Step 5: decode → resize → infer ──────────────────────────
            arr   = np.frombuffer(data, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                time.sleep(WORKER_IDLE_SLEEP_SEC)
                continue

            small = imutils_resize(frame, width=self._yolo_width)

            try:
                t0         = time.monotonic()
                results    = self._hazmat.update(small)
                elapsed_ms = (time.monotonic() - t0) * 1000
                detections = list(results) if results else []
                self.get_logger().info(
                    f'[YOLO worker] inference took {elapsed_ms:.0f} ms, '
                    f'{len(detections)} detection(s).')
            except Exception as e:
                detections = []
                print(f'[YOLO worker] inference error: {e}')

            # ── Step 6: store result and publish IMMEDIATELY ──────────────
            # Don't wait for the 5 Hz timer — publish the new result right now
            # so detections appear (and disappear) within one inference cycle.
            with self._result_lock:
                self._last_detections = detections
            self._do_publish()

            self._last_yolo_time = time.monotonic()

            # ── Step 7: yield CPU before next iteration ───────────────────
            time.sleep(WORKER_IDLE_SLEEP_SEC)

    # ── Shared publish logic (called from both YOLO thread and heartbeat timer) ──

    def _do_publish(self):
        with self._result_lock:
            detections = list(self._last_detections)

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
            m.ns                 = 'hazmat'
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
            clear.action = Marker.DELETEALL
            ma.markers.append(clear)

        self.marker_pub.publish(ma)

    # ── Heartbeat timer — main executor, keeps RViz2 markers alive ─────────

    def _publish_results(self):
        # Just re-publish whatever the YOLO thread last stored.
        # The real "new result" publish already happened on the YOLO thread.
        self._do_publish()

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