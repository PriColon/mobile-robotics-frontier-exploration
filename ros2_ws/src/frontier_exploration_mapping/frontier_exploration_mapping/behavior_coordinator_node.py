#!/usr/bin/env python3
"""
Behavior Coordinator Node
==========================
State machine that connects frontier exploration with semantic hazard avoidance.

States:
  IDLE       - waiting for frontiers
  SELECTING  - choosing safest frontier (not overlapping hazard zones)
  NAVIGATING - executing NavigateToPose goal
  ARRIVED    - goal reached, select next frontier
  DONE       - no frontiers remain, publish final status

Safety:
  - Deadman switch: if /frontier_goals stale > 5s → IDLE
  - Battery E-stop: if battery < 15% → DONE
  - Hardware E-stop: if /hazard_detection CLIFF or BUMP → pause
  - Operator kill: /exploration/stop Bool → immediate stop
  - Nav2 failure: if goal fails 3x → mark unreachable, try next

Authors: Frontier Exploration Team - ASU RAS 598 Spring 2026
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time

import numpy as np
from enum import Enum, auto
from math import sqrt

# ROS2 messages
from geometry_msgs.msg import PoseArray, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

# Nav2 action
from nav2_msgs.action import NavigateToPose

# Create3 messages
from irobot_create_msgs.msg import HazardDetectionVector


# ---------------------------------------------------------------------------
# State Machine
# ---------------------------------------------------------------------------

class State(Enum):
    IDLE       = auto()
    SELECTING  = auto()
    NAVIGATING = auto()
    ARRIVED    = auto()
    DONE       = auto()


class BehaviorCoordinator(Node):

    def __init__(self):
        super().__init__('behavior_coordinator')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('deadman_timeout_sec',  5.0)
        self.declare_parameter('battery_threshold',    0.15)
        self.declare_parameter('nav_failure_limit',    3)
        self.declare_parameter('visited_radius_m',     0.30)
        self.declare_parameter('hazard_overlap_dist',  0.50)
        self.declare_parameter('use_sim_time',         False)

        self.deadman_timeout  = self.get_parameter('deadman_timeout_sec').value
        self.battery_thresh   = self.get_parameter('battery_threshold').value
        self.nav_fail_limit   = self.get_parameter('nav_failure_limit').value
        self.visited_radius   = self.get_parameter('visited_radius_m').value
        self.hazard_overlap   = self.get_parameter('hazard_overlap_dist').value

        # ── State ──────────────────────────────────────────────────────────
        self.state              = State.IDLE
        self.frontier_goals     = []        # list of geometry_msgs/Pose
        self.last_frontier_time = None      # for deadman switch
        self.visited_frontiers  = []        # list of (x, y) already explored
        self.nav_failures       = 0
        self.current_goal       = None
        self.robot_x            = 0.0
        self.robot_y            = 0.0
        self.battery_pct        = 1.0
        self.hardware_estop     = False
        self.operator_stop      = False
        self.semantic_map       = None      # OccupancyGrid

        # ── Nav2 Action Client ────────────────────────────────────────────
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav_goal_handle = None

        # ── QoS ────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            PoseArray, '/frontier_goals',
            self._frontier_callback, 10)

        self.create_subscription(
            OccupancyGrid, '/semantic_map',
            self._semantic_map_callback, 10)

        self.create_subscription(
            Odometry, '/odometry/filtered',
            self._odom_callback, sensor_qos)

        self.create_subscription(
            BatteryState, '/battery_state',
            self._battery_callback, sensor_qos)

        self.create_subscription(
            HazardDetectionVector, '/hazard_detection',
            self._hazard_hw_callback, 10)

        self.create_subscription(
            Bool, '/exploration/stop',
            self._stop_callback, 10)

        # ── Publishers ────────────────────────────────────────────────────
        self.cmd_vel_pub    = self.create_publisher(Twist,  '/cmd_vel', 10)
        self.status_pub     = self.create_publisher(String, '/exploration/status', 10)
        self.marker_pub     = self.create_publisher(MarkerArray, '/coordinator_markers', 10)

        # ── Timer: state machine runs at 2 Hz ─────────────────────────────
        self.create_timer(0.5, self._spin_state_machine)

        self.get_logger().info('Behavior Coordinator started — state: IDLE')

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _frontier_callback(self, msg: PoseArray):
        self.frontier_goals     = list(msg.poses)
        self.last_frontier_time = self.get_clock().now()

    def _semantic_map_callback(self, msg: OccupancyGrid):
        self.semantic_map = msg

    def _odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def _battery_callback(self, msg: BatteryState):
        self.battery_pct = msg.percentage

    def _hazard_hw_callback(self, msg: HazardDetectionVector):
        # Types: CLIFF=2, BUMP=1
        types = [h.type for h in msg.detections]
        self.hardware_estop = (2 in types)   # cliff is critical

    def _stop_callback(self, msg: Bool):
        if msg.data:
            self.operator_stop = True
            self.get_logger().warn('Operator kill received — stopping exploration.')

    # ── Helpers ────────────────────────────────────────────────────────────

    def _zero_velocity(self):
        self.cmd_vel_pub.publish(Twist())

    def _dist(self, x1, y1, x2, y2):
        return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def _already_visited(self, x, y) -> bool:
        for (vx, vy) in self.visited_frontiers:
            if self._dist(x, y, vx, vy) < self.visited_radius:
                return True
        return False

    def _overlaps_hazard(self, x, y) -> bool:
        """Check if a frontier position is inside a known hazard cell."""
        if self.semantic_map is None:
            return False

        info = self.semantic_map.info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)

        if 0 <= row < info.height and 0 <= col < info.width:
            idx = row * info.width + col
            if 0 <= idx < len(self.semantic_map.data):
                cell_value = self.semantic_map.data[idx]
                return cell_value >= 70   # HAZARD threshold

        return False

    def _deadman_triggered(self) -> bool:
        if self.last_frontier_time is None:
            return False
        elapsed = (self.get_clock().now() - self.last_frontier_time).nanoseconds / 1e9
        return elapsed > self.deadman_timeout

    def _select_best_frontier(self):
        """
        Pick the highest-scored frontier that:
          - Has not been visited
          - Does not overlap a hazard cell
        Frontiers arrive pre-sorted by score (best first) from frontier_explorer_node.
        """
        for pose in self.frontier_goals:
            x = pose.position.x
            y = pose.position.y
            if self._already_visited(x, y):
                continue
            if self._overlaps_hazard(x, y):
                self.get_logger().info(
                    f'Skipping frontier ({x:.2f}, {y:.2f}) — overlaps hazard zone')
                continue
            return pose
        return None

    def _send_nav_goal(self, pose):
        """Send a NavigateToPose goal to Nav2."""
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 action server not available.')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.pose.pose            = pose

        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_callback)
        send_future.add_done_callback(self._nav_goal_response_callback)
        self.current_goal = pose
        return True

    def _nav_goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 goal rejected.')
            self.nav_failures += 1
            self.state = State.SELECTING
            return
        self._nav_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_feedback_callback(self, feedback_msg):
        pass   # Could log distance remaining here

    def _nav_result_callback(self, future):
        result = future.result()
        status = result.status

        # action_msgs/GoalStatus: SUCCEEDED=4, ABORTED=6, CANCELED=5
        if status == 4:
            self.get_logger().info('Goal SUCCEEDED.')
            if self.current_goal:
                self.visited_frontiers.append(
                    (self.current_goal.position.x,
                     self.current_goal.position.y))
            self.nav_failures = 0
            self.state = State.ARRIVED
        else:
            self.get_logger().warn(f'Goal failed with status {status}.')
            self.nav_failures += 1
            if self.nav_failures >= self.nav_fail_limit:
                self.get_logger().warn(
                    'Nav2 failure limit reached — marking frontier unreachable.')
                if self.current_goal:
                    self.visited_frontiers.append(
                        (self.current_goal.position.x,
                         self.current_goal.position.y))
                self.nav_failures = 0
            self.state = State.SELECTING

    def _cancel_navigation(self):
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None

    # ── State Machine ──────────────────────────────────────────────────────

    def _spin_state_machine(self):

        # ── Global safety checks (any state) ──────────────────────────────
        if self.operator_stop:
            self._zero_velocity()
            self._cancel_navigation()
            self._transition(State.DONE)
            self._publish_status('OPERATOR_STOP')
            return

        if self.battery_pct < self.battery_thresh:
            self._zero_velocity()
            self._cancel_navigation()
            self._transition(State.DONE)
            self._publish_status('BATTERY_LOW')
            self.get_logger().error(
                f'Battery {self.battery_pct*100:.0f}% — below threshold. Stopping.')
            return

        if self.hardware_estop and self.state == State.NAVIGATING:
            self._zero_velocity()
            self._cancel_navigation()
            self._transition(State.SELECTING)
            self._publish_status('CLIFF_DETECTED')
            return

        # ── State transitions ──────────────────────────────────────────────
        if self.state == State.IDLE:
            self._state_idle()

        elif self.state == State.SELECTING:
            self._state_selecting()

        elif self.state == State.NAVIGATING:
            self._state_navigating()

        elif self.state == State.ARRIVED:
            self._state_arrived()

        elif self.state == State.DONE:
            self._state_done()

    def _state_idle(self):
        if self.frontier_goals:
            self._transition(State.SELECTING)
        else:
            self._publish_status('IDLE_NO_FRONTIERS')

    def _state_selecting(self):
        # Deadman switch
        if self._deadman_triggered():
            self.get_logger().warn('Deadman timeout — returning to IDLE.')
            self._zero_velocity()
            self._transition(State.IDLE)
            return

        goal_pose = self._select_best_frontier()

        if goal_pose is None:
            self.get_logger().info('No valid frontiers — exploration DONE.')
            self._transition(State.DONE)
            return

        self.get_logger().info(
            f'Selected frontier: ({goal_pose.position.x:.2f}, '
            f'{goal_pose.position.y:.2f})')

        if self._send_nav_goal(goal_pose):
            self._transition(State.NAVIGATING)
        else:
            self._transition(State.IDLE)

    def _state_navigating(self):
        # Just monitor — result callback handles transitions
        self._publish_status('NAVIGATING')

    def _state_arrived(self):
        self._publish_status('ARRIVED')
        # Select next frontier immediately
        self._transition(State.SELECTING)

    def _state_done(self):
        self._zero_velocity()
        self._publish_status('EXPLORATION_COMPLETE')
        self.get_logger().info('Exploration complete.')

    # ── Utilities ─────────────────────────────────────────────────────────

    def _transition(self, new_state: State):
        self.get_logger().info(
            f'State: {self.state.name} → {new_state.name}')
        self.state = new_state

    def _publish_status(self, msg: str):
        s = String()
        s.data = msg
        self.status_pub.publish(s)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
