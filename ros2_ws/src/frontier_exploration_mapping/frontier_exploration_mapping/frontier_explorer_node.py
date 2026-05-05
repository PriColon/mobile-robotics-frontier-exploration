#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import MarkerArray, Marker

import numpy as np
from scipy.ndimage import label

import tf2_ros
from tf2_ros import TransformException


# ---------------------------------------------------------------------------
# Frontier Pipeline Functions
# ---------------------------------------------------------------------------

def find_frontiers(grid: np.ndarray):

    height, width = grid.shape
    frontier_mask = np.zeros((height, width), dtype=bool)

    free_cells = np.argwhere(grid == 0)

    for (row, col) in free_cells:
        r_min, r_max = max(0, row - 1), min(height, row + 2)
        c_min, c_max = max(0, col - 1), min(width,  col + 2)

        kernel = grid[r_min:r_max, c_min:c_max]

        has_unknown  = np.any(kernel == -1)
        has_occupied = np.any(kernel == 100)

        if has_unknown and not has_occupied:
            frontier_mask[row, col] = True

    return frontier_mask


def cluster_frontiers(frontier_mask: np.ndarray, min_cluster_size: int = 10):

    structure = np.ones((3, 3), dtype=int)
    labeled_array, num_clusters = label(frontier_mask, structure=structure)

    clusters = []

    for cluster_id in range(1, num_clusters + 1):
        cells = np.argwhere(labeled_array == cluster_id)

        if len(cells) < min_cluster_size:
            continue

        clusters.append({
            'cells':        cells,
            'size':         len(cells),
            'centroid_row': cells[:, 0].mean(),
            'centroid_col': cells[:, 1].mean(),
        })

    return clusters


def score_clusters(
    clusters: list,
    grid: np.ndarray,
    robot_row: int,
    robot_col: int,
    resolution: float,
    alpha: float = 1.0,
    beta: float  = 2.0,
    radius_meters: float = 0.5,
    epsilon: float = 1e-6
):
    height, width = grid.shape
    radius_cells  = int(radius_meters / resolution)
    scored = []

    for cluster in clusters:
        cr   = cluster['centroid_row']
        cc   = cluster['centroid_col']
        size = cluster['size']

        r_min = max(0, int(cr) - radius_cells)
        c_min = max(0, int(cc) - radius_cells)
        r_max = min(height, int(cr) + radius_cells + 1)
        c_max = min(width,  int(cc) + radius_cells + 1)
        unknown_near = int(np.sum(grid[r_min:r_max, c_min:c_max] == -1))

        dist  = np.sqrt((cr - robot_row) ** 2 + (cc - robot_col) ** 2) * resolution
        score = (alpha * size + beta * unknown_near) / (dist + epsilon)

        scored.append({
            **cluster,
            'unknown_near': unknown_near,
            'dist_meters':  dist,
            'score':        score,
        })

    scored.sort(key=lambda x: x['score'], reverse=True)
    return scored


# ---------------------------------------------------------------------------
# Helper Functions
# ---------------------------------------------------------------------------

def grid_to_world(row, col, origin, resolution):
    world_x = origin.position.x + (col + 0.5) * resolution
    world_y = origin.position.y + (row + 0.5) * resolution
    return world_x, world_y


def build_pose_array(scored_clusters, map_msg, stamp):
    pa = PoseArray()
    pa.header.frame_id = 'map'
    pa.header.stamp    = stamp

    for cluster in scored_clusters:
        wx, wy = grid_to_world(
            cluster['centroid_row'],
            cluster['centroid_col'],
            map_msg.info.origin,
            map_msg.info.resolution
        )
        pose               = Pose()
        pose.position.x    = wx
        pose.position.y    = wy
        pose.position.z    = 0.0
        pose.orientation.w = 1.0
        pa.poses.append(pose)

    return pa


def rank_to_rgb(t: float):
    if t < 0.5:
        r = 2.0 * t
        g = 1.0
    else:
        r = 1.0
        g = 1.0 - 2.0 * (t - 0.5)
    return r, g, 0.0


def build_marker_array(scored_clusters, map_msg, stamp):
    ma = MarkerArray()
    n  = len(scored_clusters)
    MAX_FRONTIERS = 1000

    res    = map_msg.info.resolution
    origin = map_msg.info.origin

    for i, cluster in enumerate(scored_clusters):
        t       = i / max(n - 1, 1)
        r, g, b = rank_to_rgb(t)

        wx, wy = grid_to_world(
            cluster['centroid_row'],
            cluster['centroid_col'],
            origin, res
        )

        # Centroid sphere
        cm                    = Marker()
        cm.header.frame_id    = 'map'
        cm.header.stamp       = stamp
        cm.ns                 = 'frontier_centroids'
        cm.id                 = i
        cm.type               = Marker.SPHERE
        cm.action             = Marker.ADD
        cm.pose.position.x    = wx
        cm.pose.position.y    = wy
        cm.pose.position.z    = 0.05
        cm.pose.orientation.w = 1.0
        cm.scale.x            = 0.20
        cm.scale.y            = 0.20
        cm.scale.z            = 0.20
        cm.color.r            = r
        cm.color.g            = g
        cm.color.b            = b
        cm.color.a            = 1.0
        cm.lifetime.sec       = 1
        cm.lifetime.nanosec   = 0
        ma.markers.append(cm)

        # Cell cloud
        cloud                    = Marker()
        cloud.header.frame_id    = 'map'
        cloud.header.stamp       = stamp
        cloud.ns                 = 'frontier_cells'
        cloud.id                 = i + MAX_FRONTIERS
        cloud.type               = Marker.POINTS
        cloud.action             = Marker.ADD
        cloud.scale.x            = res
        cloud.scale.y            = res
        cloud.color.r            = r
        cloud.color.g            = g
        cloud.color.b            = b
        cloud.color.a            = 0.6
        cloud.lifetime.sec       = 1
        cloud.lifetime.nanosec   = 0
        cloud.pose.orientation.w = 1.0

        for (row, col) in cluster['cells']:
            cx, cy = grid_to_world(row, col, origin, res)
            p = Point(); p.x = cx; p.y = cy; p.z = 0.0
            cloud.points.append(p)

        ma.markers.append(cloud)

    return ma


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('min_cluster_size', 10)
        self.declare_parameter('alpha',            1.0)
        self.declare_parameter('beta',             2.0)
        self.declare_parameter('epsilon',          1e-6)
        self.declare_parameter('radius_meters',    0.5)

        # TF2 — map-frame robot pose for accurate frontier scoring
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Cache map-frame position; updated by TF lookup inside map_callback
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        # Publishers
        self.goals_pub  = self.create_publisher(PoseArray,   '/frontier_goals',   10)
        self.marker_pub = self.create_publisher(MarkerArray, '/frontier_markers', 10)

        self.get_logger().info('Frontier Explorer Node started.')

    def _update_robot_pose(self):
        """
        Looks up map → base_link and updates self.robot_x / robot_y.
        Returns True if the lookup succeeded.
        Called once per map_callback so scoring always uses the current pose.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
            self.robot_x = tf.transform.translation.x
            self.robot_y = tf.transform.translation.y
            return True
        except TransformException:
            # TF not yet available — scoring will use the last known pose (0,0 at start)
            return False

    def map_callback(self, msg: OccupancyGrid):
        # Refresh map-frame robot pose before scoring
        self._update_robot_pose()

        grid = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))

        # Convert map-frame robot position to grid cell
        robot_col = int((self.robot_x - msg.info.origin.position.x) / msg.info.resolution)
        robot_row = int((self.robot_y - msg.info.origin.position.y) / msg.info.resolution)

        min_cluster_size = self.get_parameter('min_cluster_size').value
        alpha            = self.get_parameter('alpha').value
        beta             = self.get_parameter('beta').value
        radius_meters    = self.get_parameter('radius_meters').value
        epsilon          = self.get_parameter('epsilon').value

        frontier_mask = find_frontiers(grid)
        clusters      = cluster_frontiers(frontier_mask, min_cluster_size)
        scored        = score_clusters(clusters, grid, robot_row, robot_col,
                                       msg.info.resolution, alpha, beta,
                                       radius_meters, epsilon)

        if not scored:
            self.get_logger().info('No frontiers detected.')
            return

        stamp = self.get_clock().now().to_msg()
        self.goals_pub.publish(build_pose_array(scored, msg, stamp))
        self.marker_pub.publish(build_marker_array(scored, msg, stamp))

        self.get_logger().info(
            f'Published {len(scored)} frontiers | '
            f'best score: {scored[0]["score"]:.2f} | '
            f'dist: {scored[0]["dist_meters"]:.2f} m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()