#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose

import tf2_ros
from tf2_ros import TransformException

try:
    from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection
    HAZARD_AVAILABLE = True
except ImportError:
    HAZARD_AVAILABLE = False


FRONTIER_TIMEOUT           = 5.0
BATTERY_LOW_PCT            = 15.0
MAX_GOAL_FAILURES          = 3

PROXIMITY_LOCK_RADIUS      = 1.0
FRONTIER_SIMILARITY_RADIUS = 0.10


class State:
    IDLE       = 'IDLE'
    SELECTING  = 'SELECTING'
    PENDING    = 'PENDING'
    NAVIGATING = 'NAVIGATING'
    ARRIVED    = 'ARRIVED'
    DONE       = 'DONE'


class BehaviorCoordinator(Node):

    def __init__(self):
        super().__init__('behavior_coordinator')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('deadman_timeout_sec',  5.0)
        self.declare_parameter('battery_threshold',    0.15)
        self.declare_parameter('nav_failure_limit',    3)
        self.declare_parameter('visited_radius_m',     0.30)
        self.declare_parameter('hazard_overlap_dist',  0.50)

        self.state            = State.SELECTING
        self.frontier_goals   = []
        self.visited_set      = set()
        self.failure_counts   = {}
        self.original_goal_xy = None
        self.current_goal_xy  = None
        self.last_frontier_t  = None
        self.nav_goal_handle  = None
        self.robot_pose       = None

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.create_timer(0.05, self._tf_pose_timer, callback_group=self._cb_group)

        self.create_subscription(
            PoseArray,    '/frontier_goals',   self._frontiers_cb, 10,
            callback_group=self._cb_group)
        self.create_subscription(
            BatteryState, '/battery_state',    self._battery_cb,   10,
            callback_group=self._cb_group)
        self.create_subscription(
            Bool,         '/exploration/stop', self._kill_cb,      10,
            callback_group=self._cb_group)

        if HAZARD_AVAILABLE:
            self.create_subscription(
                HazardDetectionVector, '/hazard_detection', self._hazard_cb, 10,
                callback_group=self._cb_group)

        self._nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose',
            callback_group=self._cb_group)

        self.create_timer(0.5, self._loop,     callback_group=self._cb_group)
        self.create_timer(1.0, self._watchdog, callback_group=self._cb_group)

        self.get_logger().info('BehaviorCoordinator started — waiting for frontiers...')

    def _tf_pose_timer(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except TransformException:
            return
        self.robot_pose = (tf.transform.translation.x, tf.transform.translation.y)

    def _loop(self):
        if self.state == State.DONE:
            return
        if self.state in (State.IDLE, State.SELECTING, State.ARRIVED):
            self._try_select_and_send()

    def _find_similar_frontier(self, anchor_xy, radius):
        ax, ay = anchor_xy
        best, best_d = None, float('inf')
        for x, y in self.frontier_goals:
            if self._round_key(x, y) in self.visited_set:
                continue
            d = math.hypot(x - ax, y - ay)
            if d <= radius and d < best_d:
                best, best_d = (x, y), d
        return best

    def _dist_to_anchor(self):
        if self.robot_pose is None or self.original_goal_xy is None:
            return None
        rx, ry = self.robot_pose
        gx, gy = self.original_goal_xy
        return math.hypot(gx - rx, gy - ry)

    def _try_select_and_send(self):
        if self.state in (State.PENDING, State.NAVIGATING):
            return
        if not self.frontier_goals:
            return

        chosen = None
        for x, y in self.frontier_goals:
            if self._round_key(x, y) not in self.visited_set:
                chosen = (x, y)
                break

        if chosen is None:
            self.get_logger().info('All known frontiers visited — exploration DONE.')
            self.state = State.DONE
            return

        self.original_goal_xy = chosen
        self.current_goal_xy  = chosen
        self._send_nav_goal(*chosen)

    def _send_nav_goal(self, gx: float, gy: float):
        if not self._nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('Navigator not available — staying in SELECTING.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id    = 'map'
        goal_msg.pose.header.stamp       = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x    = gx
        goal_msg.pose.pose.position.y    = gy
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'[{self.state}] → PENDING  '
            f'anchor=({self.original_goal_xy[0]:.2f},{self.original_goal_xy[1]:.2f})  '
            f'nav_goal=({gx:.2f},{gy:.2f})')

        self.state = State.PENDING

        future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_accepted_cb)

    def _goal_accepted_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected — back to SELECTING.')
            self.state = State.SELECTING
            return

        self.get_logger().info(
            f'Goal accepted — NAVIGATING to '
            f'({self.current_goal_xy[0]:.2f},{self.current_goal_xy[1]:.2f})')
        self.state = State.NAVIGATING
        self.nav_goal_handle = handle
        handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        self.get_logger().debug(
            f'Distance remaining: {feedback_msg.feedback.distance_remaining:.2f} m')

    def _result_cb(self, future):
        from action_msgs.msg import GoalStatus
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            key = self._round_key(*self.original_goal_xy)
            self.get_logger().info(
                f'Arrived at anchor ({self.original_goal_xy[0]:.2f},'
                f'{self.original_goal_xy[1]:.2f})')
            self.visited_set.add(key)
            self.failure_counts.pop(key, None)
            self.state = State.SELECTING

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal cancelled.')
            self.state = State.SELECTING

        else:
            key = self._round_key(*self.original_goal_xy)
            self.failure_counts[key] = self.failure_counts.get(key, 0) + 1
            count = self.failure_counts[key]
            self.get_logger().warn(
                f'Goal failed (status={status}) — '
                f'failure {count}/{MAX_GOAL_FAILURES} for anchor {key}')
            if count >= MAX_GOAL_FAILURES:
                self.get_logger().warn(f'Blacklisting anchor {key}.')
                self.visited_set.add(key)
                self.failure_counts.pop(key, None)
            self.state = State.SELECTING

    def _frontiers_cb(self, msg: PoseArray):
        self.last_frontier_t = self.get_clock().now()
        self.frontier_goals  = [(p.position.x, p.position.y) for p in msg.poses]

        if self.state not in (State.NAVIGATING, State.PENDING):
            if self.state in (State.IDLE, State.SELECTING):
                self._try_select_and_send()
            return

        if self.state == State.PENDING:
            return

        if self.original_goal_xy is None:
            return

        active_key   = self._round_key(*self.current_goal_xy)
        current_keys = {self._round_key(x, y) for x, y in self.frontier_goals}

        if active_key in current_keys:
            return

        similar = self._find_similar_frontier(
            self.original_goal_xy, FRONTIER_SIMILARITY_RADIUS)

        if similar is not None:
            old = self.current_goal_xy
            self.current_goal_xy = similar
            self.get_logger().info(
                f'Frontier drifted: ({old[0]:.2f},{old[1]:.2f}) → '
                f'({similar[0]:.2f},{similar[1]:.2f})  '
                f'anchor=({self.original_goal_xy[0]:.2f},{self.original_goal_xy[1]:.2f}) '
                f'— continuing.')
            return

        dist = self._dist_to_anchor()
        dist_str = f'{dist:.2f} m' if dist is not None else 'unknown'

        if dist is not None and dist <= PROXIMITY_LOCK_RADIUS:
            self.get_logger().info(
                f'Frontier consumed, robot {dist_str} from anchor '
                f'(<= {PROXIMITY_LOCK_RADIUS} m) — holding course.')
        else:
            self.get_logger().info(
                f'Frontier consumed, robot {dist_str} from anchor '
                f'— cancelling and reselecting.')
            anchor_key = self._round_key(*self.original_goal_xy)
            self.visited_set.add(anchor_key)
            self.failure_counts.pop(anchor_key, None)
            self._cancel_active_goal()
            self.state = State.SELECTING
            self._try_select_and_send()

    def _battery_cb(self, msg: BatteryState):
        pct = msg.percentage * 100.0
        if pct < BATTERY_LOW_PCT:
            self.get_logger().warn(f'Battery low ({pct:.1f}%) — stopping.')
            self._emergency_stop()

    def _hazard_cb(self, msg):
        if not HAZARD_AVAILABLE:
            return
        for det in msg.detections:
            if det.type in (HazardDetection.BUMP, HazardDetection.CLIFF):
                self.get_logger().warn('Hazard — cancelling active goal.')
                self._cancel_active_goal()
                self.state = State.SELECTING
                return

    def _kill_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().warn('/exploration/stop — shutting down.')
            self._emergency_stop()

    def _watchdog(self):
        if self.last_frontier_t is None or self.state == State.DONE:
            return
        elapsed = (self.get_clock().now() - self.last_frontier_t).nanoseconds * 1e-9
        if elapsed > FRONTIER_TIMEOUT and self.state == State.NAVIGATING:
            self.get_logger().warn(
                f'No frontier update for {elapsed:.1f} s — reverting to IDLE.')
            self.state = State.IDLE

    def _cancel_active_goal(self):
        if self.nav_goal_handle is not None:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None

    def _emergency_stop(self):
        self._cancel_active_goal()
        self.state = State.DONE

    @staticmethod
    def _round_key(x, y, precision=1):
        return (round(x, precision), round(y, precision))


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorCoordinator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()