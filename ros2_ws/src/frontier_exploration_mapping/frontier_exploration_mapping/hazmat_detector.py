#!/usr/bin/env python3
"""
DeepHAZMAT ROS2 Node
=====================
Wraps the DeepHAZMAT YOLOv3-tiny detector as a ROS2 node.
Detects 13 categories of HAZMAT signs from the OAK-D camera feed.

Works on:
  - Laptop webcam (for testing): set camera_source:=webcam
  - Robot OAK-D camera: set camera_source:=oakd (default)

Setup:
  pip install opencv-python numpy
  git clone https://github.com/mrl-amrl/DeepHAZMAT
  cd DeepHAZMAT && pip install -e .
  # Download weights: https://github.com/mrl-amrl/DeepHAZMAT (see README)

Authors: Frontier Exploration Team - ASU RAS 598 Spring 2026
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# HAZMAT class names from DeepHAZMAT dataset
HAZMAT_CLASSES = [
    'Poison',           # 0
    'Flammable',        # 1
    'Flammable Solid',  # 2
    'Oxidizer',         # 3
    'Explosive',        # 4
    'Corrosive',        # 5
    'Radioactive',      # 6
    'Dangerous',        # 7
    'Non-Flammable Gas',# 8
    'Infectious',       # 9
    'Organic Peroxide', # 10
    'Inhalation Hazard',# 11
    'Spontaneously Combustible', # 12
]

# Severity: which classes trigger immediate E-stop
CRITICAL_CLASSES = {
    'Explosive', 'Radioactive', 'Infectious',
    'Poison', 'Inhalation Hazard'
}


class HazmatDetectorNode(Node):

    def __init__(self):
        super().__init__('hazmat_detector')

        # Parameters
        self.declare_parameter('camera_source', 'oakd')     # 'oakd' or 'webcam'
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('weights_path', '')           # path to .weights file
        self.declare_parameter('config_path', '')            # path to .cfg file
        self.declare_parameter('use_sim_time', False)

        self.camera_source  = self.get_parameter('camera_source').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.weights_path   = self.get_parameter('weights_path').value
        self.config_path    = self.get_parameter('config_path').value

        self.bridge = CvBridge()
        self.net    = None
        self._webcam = None
        self._latest_frame = None

        # Load YOLO model
        self._load_model()

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Subscribers
        if self.camera_source == 'oakd':
            self.create_subscription(
                Image, '/oakd/rgb/preview/image_raw',
                self._image_callback, sensor_qos)
            self.get_logger().info('Using OAK-D camera feed.')
        else:
            # Webcam mode — open camera and use timer
            self._webcam = cv2.VideoCapture(0)
            self.get_logger().info('Using laptop webcam (device 0).')

        # Publishers
        self.detection_pub  = self.create_publisher(String,      '/hazmat_detection', 10)
        self.marker_pub     = self.create_publisher(MarkerArray,  '/hazmat_markers', 10)
        self.annotated_pub  = self.create_publisher(Image,        '/hazmat/annotated_image', 10)

        # Processing timer — 5 Hz (YOLO is slow on CPU)
        self.create_timer(0.2, self._process)

        self.get_logger().info('HAZMAT Detector Node started.')

    def _load_model(self):
        """Load YOLOv3-tiny model."""
        if not self.weights_path or not self.config_path:
            self.get_logger().warn(
                'No weights/config path provided. '
                'Running in DEMO mode (colour-based HAZMAT cue only).')
            return

        try:
            self.net = cv2.dnn.readNet(self.weights_path, self.config_path)
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            self.get_logger().info('YOLOv3-tiny model loaded successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            self.net = None

    def _image_callback(self, msg: Image):
        try:
            self._latest_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion error: {e}')

    def _get_frame(self):
        """Get current frame from either OAK-D or webcam."""
        if self.camera_source == 'webcam':
            if self._webcam is None:
                return None
            ret, frame = self._webcam.read()
            return frame if ret else None
        else:
            return self._latest_frame

    def _process(self):
        frame = self._get_frame()
        if frame is None:
            return

        detections = []

        if self.net is not None:
            detections = self._run_yolo(frame)
        else:
            # Demo mode: detect HAZMAT diamond shape by colour
            detections = self._demo_colour_detect(frame)

        # Annotate and publish image
        annotated = self._draw_detections(frame.copy(), detections)
        try:
            img_msg = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
            self.annotated_pub.publish(img_msg)
        except Exception:
            pass

        # Publish detection results
        if detections:
            names = [d['class'] for d in detections]
            msg = String()
            msg.data = ','.join(names)
            self.detection_pub.publish(msg)

            self.get_logger().info(f'HAZMAT detected: {names}')

            # Publish markers
            self._publish_markers(detections)

    def _run_yolo(self, frame: np.ndarray) -> list:
        """Run YOLOv3-tiny inference."""
        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(
            frame, 1/255.0, (416, 416),
            swapRB=True, crop=False)

        self.net.setInput(blob)

        # Get output layer names
        out_layers = self.net.getUnconnectedOutLayersNames()
        outputs = self.net.forward(out_layers)

        detections = []
        boxes, confidences, class_ids = [], [], []

        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = int(np.argmax(scores))
                confidence = float(scores[class_id])

                if confidence < self.conf_threshold:
                    continue

                cx = int(detection[0] * w)
                cy = int(detection[1] * h)
                bw = int(detection[2] * w)
                bh = int(detection[3] * h)
                x  = cx - bw // 2
                y  = cy - bh // 2

                boxes.append([x, y, bw, bh])
                confidences.append(confidence)
                class_ids.append(class_id)

        # Non-maximum suppression
        indices = cv2.dnn.NMSBoxes(
            boxes, confidences, self.conf_threshold, 0.4)

        if indices is not None and len(indices) > 0:
            for i in indices.flatten():
                x, y, bw, bh = boxes[i]
                class_name = (HAZMAT_CLASSES[class_ids[i]]
                              if class_ids[i] < len(HAZMAT_CLASSES)
                              else f'class_{class_ids[i]}')
                detections.append({
                    'class': class_name,
                    'confidence': confidences[i],
                    'bbox': (x, y, bw, bh),
                    'critical': class_name in CRITICAL_CLASSES,
                })

        return detections

    def _demo_colour_detect(self, frame: np.ndarray) -> list:
        """
        Demo mode: detect HAZMAT diamond shapes by colour cue.
        Orange = flammable, yellow = corrosive, blue = inhalation
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]
        detections = []

        # Orange patch → Flammable
        orange = cv2.inRange(hsv,
            np.array([5, 150, 150]),
            np.array([20, 255, 255]))
        if cv2.countNonZero(orange) > 0.05 * h * w:
            detections.append({
                'class': 'Flammable',
                'confidence': 0.6,
                'bbox': (0, 0, w, h),
                'critical': False,
            })

        # Yellow patch → Corrosive
        yellow = cv2.inRange(hsv,
            np.array([20, 100, 150]),
            np.array([35, 255, 255]))
        if cv2.countNonZero(yellow) > 0.05 * h * w:
            detections.append({
                'class': 'Corrosive',
                'confidence': 0.6,
                'bbox': (0, 0, w, h),
                'critical': False,
            })

        return detections

    def _draw_detections(self, frame: np.ndarray, detections: list) -> np.ndarray:
        """Draw bounding boxes and labels on frame."""
        for d in detections:
            x, y, bw, bh = d['bbox']
            color = (0, 0, 255) if d['critical'] else (0, 165, 255)
            label = f"{d['class']} {d['confidence']:.2f}"

            cv2.rectangle(frame, (x, y), (x + bw, y + bh), color, 2)
            cv2.putText(frame, label, (x, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            if d['critical']:
                cv2.putText(frame, '! CRITICAL !', (x, y + bh + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        return frame

    def _publish_markers(self, detections: list):
        """Publish RViz2 markers for detected HAZMAT signs."""
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, d in enumerate(detections):
            m = Marker()
            m.header.frame_id = 'base_link'
            m.header.stamp    = stamp
            m.ns              = 'hazmat_detections'
            m.id              = i
            m.type            = Marker.CUBE
            m.action          = Marker.ADD

            m.pose.position.x = 1.0   # ~1m in front of robot
            m.pose.position.y = 0.0
            m.pose.position.z = 0.5
            m.pose.orientation.w = 1.0

            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.3

            # Critical = red, warning = orange
            if d['critical']:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 1.0
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 1.0

            m.lifetime.sec     = 2
            m.lifetime.nanosec = 0
            ma.markers.append(m)

        self.marker_pub.publish(ma)

    def destroy_node(self):
        if self._webcam is not None:
            self._webcam.release()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = HazmatDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
