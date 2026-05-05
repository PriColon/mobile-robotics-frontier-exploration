#!/usr/bin/env python3

import math
import heapq
import threading
import time
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

from scipy.ndimage import binary_dilation, distance_transform_edt
import tf2_ros
from tf2_ros import TransformException
from nav2_msgs.action import NavigateToPose

try:
    from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection
    HAZARD_AVAILABLE = True
except ImportError:
    HAZARD_AVAILABLE = False


V_MAX = 0.20
V_MIN = 0.05
W_MAX = 0.80

KP_LINEAR  = 0.60
KP_ANGULAR = 0.40

ALIGN_THRESHOLD    = 0.10
REALIGN_THRESHOLD  = 0.35
WAYPOINT_THRESHOLD = 0.25
GOAL_THRESHOLD     = 0.35

CONTROL_HZ = 20
CONTROL_DT = 1.0 / CONTROL_HZ

RECOMPUTE_WAYPOINTS_REMAINING = 2

ROBOT_HALF_WIDTH = 0.20

GRID_RESOLUTION    = 0.10
SNAP_SEARCH_RADIUS = 10

NOISE_KERNEL_PX = 3
WALL_DILATION_M = 0.10
HARD_RADIUS_M   = 0.20
SOFT_RADIUS_M   = 0.25

COST_FREE  =    0.0
COST_SOFT  =   10.0
COST_HARD  =   70.0
COST_WALL  = 1000.0

_NEIGHBOURS = [
    ( 0,  1, 1.0), ( 0, -1, 1.0), ( 1,  0, 1.0), (-1,  0, 1.0),
    ( 1,  1, math.sqrt(2)), ( 1, -1, math.sqrt(2)),
    (-1,  1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
]


class DriveState(Enum):
    IDLE    = auto()
    TURNING = auto()
    DRIVING = auto()


def world_to_cell(wx: float, wy: float, ox: float, oy: float, res: float):
    return int((wx - ox) / res), int((wy - oy) / res)


def cell_to_world(col: int, row: int, ox: float, oy: float, res: float):
    return ox + (col + 0.5) * res, oy + (row + 0.5) * res


def build_costmap(occ_msg: OccupancyGrid, grid_res: float):
    info   = occ_msg.info
    res    = info.resolution
    slam_w = info.width
    slam_h = info.height
    ox     = info.origin.position.x
    oy     = info.origin.position.y

    raw  = np.array(occ_msg.data, dtype=np.int8).reshape((slam_h, slam_w))

    free = raw == 0
    wall = raw >= 50
    unkn = raw == -1

    noise_kernel = np.ones((NOISE_KERNEL_PX, NOISE_KERNEL_PX), dtype=bool)
    clean_free   = binary_dilation(free, structure=noise_kernel) & ~wall & ~unkn

    dil_px       = max(1, int(round(WALL_DILATION_M / res)))
    dil_kernel   = np.ones((2 * dil_px + 1,) * 2, dtype=bool)
    dilated_wall = binary_dilation(wall, structure=dil_kernel)

    impassable = dilated_wall | unkn

    dist_from_wall = distance_transform_edt(~dilated_wall)
    hard_px = HARD_RADIUS_M / res
    soft_px = hard_px + SOFT_RADIUS_M / res

    cost        = np.full((slam_h, slam_w), COST_WALL, dtype=np.float32)
    traversable = clean_free & ~impassable

    cost[traversable & (dist_from_wall >  soft_px)] = COST_FREE
    cost[traversable & (dist_from_wall <= soft_px)] = COST_SOFT
    cost[traversable & (dist_from_wall <= hard_px)] = COST_HARD
    cost[impassable]                                = COST_WALL

    gw = max(1, int(round(slam_w * res / grid_res)))
    gh = max(1, int(round(slam_h * res / grid_res)))
    src_cols = np.clip((np.arange(gw) * grid_res / res).astype(int), 0, slam_w - 1)
    src_rows = np.clip((np.arange(gh) * grid_res / res).astype(int), 0, slam_h - 1)
    grid = cost[np.ix_(src_rows, src_cols)]

    return grid, ox, oy, gw, gh


def snap_to_free(grid: np.ndarray, col: int, row: int, gw: int, gh: int,
                 max_r: int = SNAP_SEARCH_RADIUS):
    def navigable(c, r):
        return 0 <= c < gw and 0 <= r < gh and float(grid[r, c]) < COST_WALL

    if navigable(col, row):
        return col, row

    for radius in range(1, max_r + 1):
        for dc in range(-radius, radius + 1):
            for dr in range(-radius, radius + 1):
                if abs(dc) == radius or abs(dr) == radius:
                    if navigable(col + dc, row + dr):
                        return col + dc, row + dr
    return None


def astar(grid: np.ndarray, start: tuple, goal: tuple, gw: int, gh: int):
    def cell_cost(c, r):
        if 0 <= c < gw and 0 <= r < gh:
            return float(grid[r, c])
        return COST_WALL

    open_heap = [(math.hypot(goal[0] - start[0], goal[1] - start[1]), 0.0, start)]
    g_cost    = {start: 0.0}
    came_from = {}

    while open_heap:
        _, g, cur = heapq.heappop(open_heap)

        if cur == goal:
            path = []
            while cur in came_from:
                path.append(cur)
                cur = came_from[cur]
            path.append(start)
            return list(reversed(path))

        if g > g_cost.get(cur, float('inf')):
            continue

        for dc, dr, step_cost in _NEIGHBOURS:
            nb   = (cur[0] + dc, cur[1] + dr)
            nb_c = cell_cost(*nb)

            if nb_c >= COST_WALL and nb != goal:
                continue

            new_g = g + step_cost + nb_c
            if new_g < g_cost.get(nb, float('inf')):
                g_cost[nb]    = new_g
                came_from[nb] = cur
                h = math.hypot(goal[0] - nb[0], goal[1] - nb[1])
                heapq.heappush(open_heap, (new_g + h, new_g, nb))

    return None


def _swept_corridor_clear(grid: np.ndarray,
                           wx1: float, wy1: float,
                           wx2: float, wy2: float,
                           ox: float, oy: float,
                           res: float, gw: int, gh: int) -> bool:
    dx, dy  = wx2 - wx1, wy2 - wy1
    seg_len = math.hypot(dx, dy)
    if seg_len < 1e-6:
        return True

    fwd_x, fwd_y   = dx / seg_len, dy / seg_len
    perp_x, perp_y = -fwd_y, fwd_x
    hw = ROBOT_HALF_WIDTH

    corners = [
        (wx1 + perp_x * hw, wy1 + perp_y * hw),
        (wx1 - perp_x * hw, wy1 - perp_y * hw),
        (wx2 + perp_x * hw, wy2 + perp_y * hw),
        (wx2 - perp_x * hw, wy2 - perp_y * hw),
    ]
    cs = [world_to_cell(wx, wy, ox, oy, res) for wx, wy in corners]

    col_min = max(0,    min(c for c, _ in cs))
    col_max = min(gw-1, max(c for c, _ in cs))
    row_min = max(0,    min(r for _, r in cs))
    row_max = min(gh-1, max(r for _, r in cs))

    if col_min > col_max or row_min > row_max:
        return True

    mid_wx   = (wx1 + wx2) * 0.5
    mid_wy   = (wy1 + wy2) * 0.5
    half_len = seg_len * 0.5

    cols = np.arange(col_min, col_max + 1)
    rows = np.arange(row_min, row_max + 1)
    cc, rr = np.meshgrid(cols, rows)

    cell_wx = ox + (cc + 0.5) * res
    cell_wy = oy + (rr + 0.5) * res
    rel_x   = cell_wx - mid_wx
    rel_y   = cell_wy - mid_wy

    inside = (
        (np.abs(rel_x * fwd_x  + rel_y * fwd_y)  <= half_len) &
        (np.abs(rel_x * perp_x + rel_y * perp_y) <= hw)
    )

    if not np.any(inside):
        return True

    return bool(np.all(grid[rr[inside], cc[inside]] < COST_WALL))


def prune_path(grid: np.ndarray,
               path: list,
               ox: float, oy: float,
               res: float, gw: int, gh: int) -> list:
    if len(path) < 2:
        return list(path)

    pruned = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1:
            if _swept_corridor_clear(grid, *path[i], *path[j], ox, oy, res, gw, gh):
                break
            j -= 1
        pruned.append(path[j])
        i = j

    return pruned


def _make_marker(ns: str, mid: int, mtype: int, stamp, lifetime_ns: int = 0) -> Marker:
    m = Marker()
    m.header.frame_id    = 'map'
    m.header.stamp       = stamp
    m.ns, m.id, m.type   = ns, mid, mtype
    m.action             = Marker.ADD
    m.pose.orientation.w = 1.0
    m.lifetime           = Duration(sec=0, nanosec=lifetime_ns)
    return m


def build_path_markers(raw_wps: list, pruned_wps: list, goal_xy: tuple, stamp) -> MarkerArray:
    arr = MarkerArray()

    if len(raw_wps) > 1:
        m = _make_marker('raw_astar', 0, Marker.LINE_STRIP, stamp)
        m.scale.x = 0.02
        m.color.r = m.color.g = m.color.b = 1.0
        m.color.a = 0.35
        m.points  = [Point(x=wx, y=wy, z=0.02) for wx, wy in raw_wps]
        arr.markers.append(m)

    if len(pruned_wps) > 1:
        m = _make_marker('pruned_path', 0, Marker.LINE_STRIP, stamp)
        m.scale.x = 0.07
        m.color.r = 0.0; m.color.g = 0.85; m.color.b = 1.0; m.color.a = 0.9
        m.points  = [Point(x=wx, y=wy, z=0.05) for wx, wy in pruned_wps]
        arr.markers.append(m)

    for i, (wx, wy) in enumerate(pruned_wps or []):
        s = _make_marker('wp_spheres', i * 2, Marker.SPHERE, stamp)
        s.pose.position.x = wx; s.pose.position.y = wy; s.pose.position.z = 0.06
        s.scale.x = s.scale.y = s.scale.z = 0.10
        s.color.r = 1.0; s.color.g = 0.9; s.color.b = 0.0; s.color.a = 0.85
        arr.markers.append(s)

        t = _make_marker('wp_labels', i * 2 + 1, Marker.TEXT_VIEW_FACING, stamp)
        t.pose.position.x = wx; t.pose.position.y = wy; t.pose.position.z = 0.20
        t.scale.z = 0.12; t.color.r = t.color.g = t.color.b = t.color.a = 1.0
        t.text = str(i)
        arr.markers.append(t)

    if goal_xy:
        g = _make_marker('goal_marker', 0, Marker.SPHERE, stamp)
        g.pose.position.x, g.pose.position.y = goal_xy
        g.pose.position.z = 0.10
        g.scale.x = g.scale.y = g.scale.z = 0.30
        g.color.r = 1.0; g.color.g = 0.0; g.color.b = 1.0; g.color.a = 1.0
        arr.markers.append(g)

    return arr


def build_active_markers(tx: float, ty: float,
                          rx: float, ry: float,
                          wp_idx: int, total_wps: int,
                          stamp) -> MarkerArray:
    arr = MarkerArray()
    lt  = 150_000_000

    tgt = _make_marker('active_target', 0, Marker.SPHERE, stamp, lt)
    tgt.pose.position.x = tx; tgt.pose.position.y = ty; tgt.pose.position.z = 0.18
    tgt.scale.x = tgt.scale.y = tgt.scale.z = 0.22
    tgt.color.r = 1.0; tgt.color.g = 0.45; tgt.color.b = 0.0; tgt.color.a = 1.0
    arr.markers.append(tgt)

    rob = _make_marker('robot_pos', 0, Marker.SPHERE, stamp, lt)
    rob.pose.position.x = rx; rob.pose.position.y = ry; rob.pose.position.z = 0.15
    rob.scale.x = rob.scale.y = rob.scale.z = 0.14
    rob.color.r = rob.color.g = rob.color.b = 1.0; rob.color.a = 0.9
    arr.markers.append(rob)

    lbl = _make_marker('active_label', 0, Marker.TEXT_VIEW_FACING, stamp, lt)
    lbl.pose.position.x = tx; lbl.pose.position.y = ty; lbl.pose.position.z = 0.35
    lbl.scale.z = 0.14
    lbl.color.r = lbl.color.g = lbl.color.b = lbl.color.a = 1.0
    if wp_idx >= total_wps:
        lbl.text = f'GOAL ({tx:.1f},{ty:.1f})'
    else:
        lbl.text = f'WP {wp_idx}/{total_wps - 1} ({tx:.1f},{ty:.1f})'
    arr.markers.append(lbl)

    return arr


def build_costmap_msg(grid: np.ndarray, ox: float, oy: float,
                      gw: int, gh: int, stamp) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header.frame_id             = 'map'
    msg.header.stamp                = stamp
    msg.info.resolution             = GRID_RESOLUTION
    msg.info.width, msg.info.height = gw, gh
    msg.info.origin.position.x      = ox
    msg.info.origin.position.y      = oy
    msg.info.origin.orientation.w   = 1.0
    scaled = np.clip(grid * (100.0 / COST_HARD), 0.0, 100.0).astype(np.int8)
    scaled[grid >= COST_WALL] = 100
    msg.data = scaled.flatten().tolist()
    return msg


def build_costmap_debug_markers(grid: np.ndarray, ox: float, oy: float,
                                gw: int, gh: int, stamp) -> MarkerArray:
    arr = MarkerArray()
    res = GRID_RESOLUTION

    cols = np.arange(gw)
    rows = np.arange(gh)
    cc, rr = np.meshgrid(cols, rows)
    cell_wx = (ox + (cc + 0.5) * res).flatten()
    cell_wy = (oy + (rr + 0.5) * res).flatten()
    flat    = grid.flatten()

    tiers = [
        ('costmap_free', 0, COST_FREE, COST_FREE, 0.0, 1.0, 0.0, 0.5),
        ('costmap_soft', 1, COST_SOFT, COST_SOFT, 1.0, 1.0, 0.0, 0.7),
        ('costmap_hard', 2, COST_HARD, COST_HARD, 1.0, 0.0, 0.0, 0.8),
    ]

    for ns, mid, cost_lo, cost_hi, cr, cg, cb, ca in tiers:
        mask = (flat >= cost_lo - 1e-3) & (flat <= cost_hi + 1e-3) & (flat < COST_WALL)
        if not np.any(mask):
            continue

        m = _make_marker(ns, mid, Marker.POINTS, stamp)
        m.scale.x = res * 0.9
        m.scale.y = res * 0.9
        m.color.r = cr; m.color.g = cg; m.color.b = cb; m.color.a = ca

        xs = cell_wx[mask]
        ys = cell_wy[mask]
        m.points = [Point(x=float(x), y=float(y), z=0.01) for x, y in zip(xs, ys)]
        arr.markers.append(m)

    return arr


def _direction_changes(cells: list) -> list:
    if len(cells) < 2:
        return list(cells)

    candidates = [cells[0]]
    prev_d = (cells[1][0] - cells[0][0], cells[1][1] - cells[0][1])

    for k in range(1, len(cells) - 1):
        d = (cells[k + 1][0] - cells[k][0], cells[k + 1][1] - cells[k][1])
        if d != prev_d:
            candidates.append(cells[k])
            prev_d = d

    candidates.append(cells[-1])
    return candidates


class SimpleNavigatorNode(Node):

    def __init__(self):
        super().__init__('simple_navigator')

        self._cb_group = ReentrantCallbackGroup()

        self.pose     = None
        self.map_msg  = None
        self._pose_lock = threading.Lock()
        self._map_lock  = threading.Lock()

        self.estop = False

        self._cancel_event = threading.Event()
        self._executing    = False
        self._exec_lock    = threading.Lock()
        self._active_goal  = None

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.create_timer(0.05, self._tf_pose_timer, callback_group=self._cb_group)

        self.create_subscription(OccupancyGrid, '/map',
                                 self._map_cb, 10,
                                 callback_group=self._cb_group)
        self.create_subscription(Bool, '/exploration/stop',
                                 self._kill_cb, 10,
                                 callback_group=self._cb_group)
        if HAZARD_AVAILABLE:
            self.create_subscription(HazardDetectionVector, '/hazard_detection',
                                     self._hazard_cb, 10,
                                     callback_group=self._cb_group)

        self.cmd_vel_pub         = self.create_publisher(Twist,         '/cmd_vel_unstamped',  10)
        self.path_marker_pub     = self.create_publisher(MarkerArray,   '/nav_markers',        10)
        self.active_marker_pub   = self.create_publisher(MarkerArray,   '/nav_markers_active', 10)
        self.costmap_pub         = self.create_publisher(OccupancyGrid, '/nav_costmap',        10)
        self.costmap_debug_pub   = self.create_publisher(MarkerArray,   '/nav_costmap_debug',  10)

        self._last_costmap: tuple | None = None
        self.create_timer(2.0, self._republish_costmap_timer, callback_group=self._cb_group)

        self._action_server = ActionServer(
            self, NavigateToPose, '/navigate_to_pose',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group,
        )
        self.get_logger().info('SimpleNavigator ready.')

    def _goal_cb(self, goal_request):
        new_gx = goal_request.pose.pose.position.x
        new_gy = goal_request.pose.pose.position.y
        with self._exec_lock:
            if self._executing and self._active_goal is not None:
                ax, ay = self._active_goal
                if math.hypot(new_gx - ax, new_gy - ay) < GOAL_THRESHOLD:
                    return GoalResponse.REJECT
                self._cancel_event.set()
        self.get_logger().info('Goal received — accepting.')
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _):
        self._cancel_event.set()
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle: ServerGoalHandle):
        gx = goal_handle.request.pose.pose.position.x
        gy = goal_handle.request.pose.pose.position.y
        with self._exec_lock:
            self._executing   = True
            self._active_goal = (gx, gy)
        self._cancel_event.clear()
        try:
            return self._run(goal_handle)
        finally:
            with self._exec_lock:
                self._executing   = False
                self._active_goal = None

    def _run(self, goal_handle: ServerGoalHandle):
        gx = goal_handle.request.pose.pose.position.x
        gy = goal_handle.request.pose.pose.position.y
        self.get_logger().info(f'Navigating to ({gx:.2f}, {gy:.2f})')

        if not self._wait_for_map_and_pose(timeout=10.0):
            return self._abort(goal_handle, 'No map/pose within 10 s — cannot plan')

        plan = self._compute_plan(gx, gy)
        if plan is None:
            return self._abort(goal_handle, 'Initial planning failed')

        waypoints, raw_wps, pgrid, pox, poy, pgw, pgh = plan
        self._publish_path_markers(raw_wps, waypoints, (gx, gy))
        self._publish_costmap(pgrid, pox, poy, pgw, pgh)

        wp_idx      = 1 if len(waypoints) > 1 else 0
        drive_state = DriveState.TURNING

        NO_DRIVE_TIMEOUT = 6.0
        no_drive_accum   = 0.0
        ever_drove       = False

        _recompute_done = False

        while True:
            if self._cancel_event.is_set():
                self._stop_robot()
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return NavigateToPose.Result()

            if self.estop:
                return self._abort(goal_handle, 'E-stop triggered')

            with self._pose_lock:
                pose = self.pose
            if pose is None:
                time.sleep(CONTROL_DT)
                continue

            rx, ry, ryaw = pose

            dist_to_goal = math.hypot(gx - rx, gy - ry)
            if dist_to_goal < GOAL_THRESHOLD:
                self._stop_robot()
                goal_handle.succeed()
                self.get_logger().info(f'Goal reached (dist={dist_to_goal:.3f} m)')
                return NavigateToPose.Result()

            if drive_state != DriveState.DRIVING:
                no_drive_accum += CONTROL_DT
            else:
                no_drive_accum = 0.0
                ever_drove     = True

            if no_drive_accum >= NO_DRIVE_TIMEOUT:
                return self._abort(
                    goal_handle,
                    f'No drive progress for {NO_DRIVE_TIMEOUT:.0f} s '
                    f'(stuck in {drive_state.name}) — frontier unviable'
                )

            remaining = len(waypoints) - wp_idx
            if wp_idx > 1 and remaining <= RECOMPUTE_WAYPOINTS_REMAINING and not _recompute_done:
                _recompute_done = True
                self.get_logger().info('Near end of path — refreshing costmap and replanning.')
                plan = self._compute_plan(gx, gy)
                if plan is not None:
                    waypoints, raw_wps, pgrid, pox, poy, pgw, pgh = plan
                    wp_idx          = 1 if len(waypoints) > 1 else 0
                    drive_state     = DriveState.TURNING
                    _recompute_done = False
                    self._publish_path_markers(raw_wps, waypoints, (gx, gy))
                    self._publish_costmap(pgrid, pox, poy, pgw, pgh)

            while wp_idx < len(waypoints):
                wx, wy = waypoints[wp_idx]
                if math.hypot(wx - rx, wy - ry) < WAYPOINT_THRESHOLD:
                    self.get_logger().info(f'Waypoint {wp_idx} reached — advancing.')
                    wp_idx        += 1
                    drive_state    = DriveState.TURNING
                    no_drive_accum = 0.0
                else:
                    break

            if wp_idx >= len(waypoints):
                tx, ty = gx, gy
            else:
                tx, ty = waypoints[wp_idx]

            desired_yaw   = math.atan2(ty - ry, tx - rx)
            heading_error = math.atan2(
                math.sin(desired_yaw - ryaw),
                math.cos(desired_yaw - ryaw)
            )
            dist_to_wp = math.hypot(tx - rx, ty - ry)

            if drive_state == DriveState.TURNING:
                if abs(heading_error) < ALIGN_THRESHOLD:
                    drive_state = DriveState.DRIVING
                    v, w = 0.0, 0.0
                else:
                    v = 0.0
                    w = max(-W_MAX, min(W_MAX, KP_ANGULAR * heading_error))
                    if w != 0.0 and abs(w) < 0.10:
                        w = math.copysign(0.10, w)

            elif drive_state == DriveState.DRIVING:
                if abs(heading_error) > REALIGN_THRESHOLD:
                    drive_state = DriveState.TURNING
                    v, w = 0.0, 0.0
                else:
                    v = max(V_MIN, min(V_MAX, KP_LINEAR * dist_to_wp))
                    w = max(-W_MAX, min(W_MAX, KP_ANGULAR * heading_error))

            else:
                v, w = 0.0, 0.0

            self._publish_velocity(v, w)

            stamp = self.get_clock().now().to_msg()
            self.active_marker_pub.publish(
                build_active_markers(tx, ty, rx, ry, wp_idx, len(waypoints), stamp)
            )
            fb = NavigateToPose.Feedback()
            fb.distance_remaining = dist_to_goal
            goal_handle.publish_feedback(fb)

            time.sleep(CONTROL_DT)

    def _compute_plan(self, gx: float, gy: float):
        with self._map_lock:
            if self.map_msg is None:
                self.get_logger().error('_compute_plan: no map available')
                return None
            map_snap = self.map_msg

        with self._pose_lock:
            pose = self.pose
        if pose is None:
            self.get_logger().error('_compute_plan: no pose available')
            return None

        rx, ry, _ = pose

        grid, ox, oy, gw, gh = build_costmap(map_snap, GRID_RESOLUTION)

        def clamped_cell(wx, wy):
            c, r = world_to_cell(wx, wy, ox, oy, GRID_RESOLUTION)
            return max(0, min(c, gw - 1)), max(0, min(r, gh - 1))

        raw_start = clamped_cell(rx, ry)
        raw_goal  = clamped_cell(gx, gy)

        start_cell = snap_to_free(grid, *raw_start, gw, gh)
        goal_cell  = snap_to_free(grid, *raw_goal,  gw, gh)

        if start_cell is None:
            self.get_logger().error(f'Start {raw_start} is surrounded by obstacles — cannot plan')
            return None
        if goal_cell is None:
            self.get_logger().error(f'Goal {raw_goal} is surrounded by obstacles — cannot plan')
            return None

        self.get_logger().info(
            f'Planning: world ({rx:.2f},{ry:.2f})→({gx:.2f},{gy:.2f}) '
            f'cells {start_cell}→{goal_cell}'
        )

        path_cells = astar(grid, start_cell, goal_cell, gw, gh)
        if path_cells is None:
            self.get_logger().warn('A*: no path found')
            return None

        raw_wps   = [cell_to_world(c, r, ox, oy, GRID_RESOLUTION) for c, r in path_cells]
        turn_cells = _direction_changes(path_cells)
        turn_world = [cell_to_world(c, r, ox, oy, GRID_RESOLUTION) for c, r in turn_cells]
        waypoints  = prune_path(grid, turn_world, ox, oy, GRID_RESOLUTION, gw, gh)

        self.get_logger().info(
            f'{len(raw_wps)} raw → {len(turn_world)} turns → {len(waypoints)} pruned waypoints'
        )
        return waypoints, raw_wps, grid, ox, oy, gw, gh

    def _tf_pose_timer(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
        except TransformException:
            return
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        with self._pose_lock:
            self.pose = (t.x, t.y, yaw)

    def _wait_for_map_and_pose(self, timeout: float = 10.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._pose_lock:
                have_pose = self.pose is not None
            if self.map_msg is not None and have_pose:
                return True
            time.sleep(0.1)
        return False

    def _map_cb(self, msg: OccupancyGrid):
        with self._map_lock:
            self.map_msg = msg

    def _hazard_cb(self, msg: HazardDetectionVector):
        for det in msg.detections:
            if det.type in (HazardDetection.BUMP, HazardDetection.CLIFF):
                if not self.estop:
                    self.get_logger().warn(f'Hazard type={det.type} — engaging e-stop')
                self.estop = True
                self._stop_robot()
                return
        self.estop = False

    def _kill_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().warn('/exploration/stop received — halting navigation')
            self.estop = True
            self._cancel_event.set()
            self._stop_robot()

    def _abort(self, goal_handle: ServerGoalHandle, reason: str):
        self.get_logger().warn(f'Aborting: {reason}')
        self._stop_robot()
        goal_handle.abort()
        return NavigateToPose.Result()

    def _stop_robot(self):
        self._publish_velocity(0.0, 0.0)

    def _publish_velocity(self, v: float, w: float):
        msg = Twist()
        msg.linear.x  = float(v)
        msg.angular.z = float(w)
        self.cmd_vel_pub.publish(msg)

    def _publish_path_markers(self, raw_wps, pruned_wps, goal_xy):
        stamp = self.get_clock().now().to_msg()
        self.path_marker_pub.publish(
            build_path_markers(raw_wps, pruned_wps, goal_xy, stamp)
        )

    def _publish_costmap(self, grid, ox, oy, gw, gh):
        stamp = self.get_clock().now().to_msg()
        self.costmap_pub.publish(build_costmap_msg(grid, ox, oy, gw, gh, stamp))
        self.costmap_debug_pub.publish(build_costmap_debug_markers(grid, ox, oy, gw, gh, stamp))
        self._last_costmap = (grid, ox, oy, gw, gh)

    def _republish_costmap_timer(self):
        if self._last_costmap is None:
            return
        grid, ox, oy, gw, gh = self._last_costmap
        stamp = self.get_clock().now().to_msg()
        self.costmap_pub.publish(build_costmap_msg(grid, ox, oy, gw, gh, stamp))
        self.costmap_debug_pub.publish(build_costmap_debug_markers(grid, ox, oy, gw, gh, stamp))


def main(args=None):
    rclpy.init(args=args)
    node     = SimpleNavigatorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()