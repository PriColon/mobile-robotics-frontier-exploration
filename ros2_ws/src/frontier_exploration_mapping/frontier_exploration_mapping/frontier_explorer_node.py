import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header

import numpy as np
from scipy.ndimage import label


# ---------------------------------------------------------------------------
# Frontier Pipeline Functions
# ---------------------------------------------------------------------------

def find_frontiers(grid: np.ndarray):

    height, width = grid.shape
    frontier_mask = np.zeros((height, width), dtype=bool)

    # Grab only free cells
    free_cells = np.argwhere(grid == 0)

    # Go through each free cell and evalute frontier status using kernel
    for (row, col) in free_cells:

        # Store and clamp surrounding cell indexes of current free cell
        r_min, r_max = max(0, row - 1), min(height, row + 2)
        c_min, c_max = max(0, col - 1), min(width,  col + 2)

        # Store cells using indexes
        kernel = grid[r_min:r_max, c_min:c_max]

        # Check status of cells in kernel
        has_unknown  = np.any(kernel == -1)
        has_occupied = np.any(kernel == 100)

        # Evaluate if cell satisfies frontier conditions
        if has_unknown and not has_occupied:
            frontier_mask[row, col] = True

    return frontier_mask

def cluster_frontiers(frontier_mask: np.ndarray, min_cluster_size: int = 10):

    # Label cells touching eachother under a single ID
    structure = np.ones((3, 3), dtype=int)
    labeled_array, num_clusters = label(frontier_mask, structure=structure)

    clusters = []

    # Store cells with same ID into cluster list
    for cluster_id in range(1, num_clusters + 1):   
        cells = np.argwhere(labeled_array == cluster_id) # Store idxs with label

        if len(cells) < min_cluster_size:
            continue    # discard noise

        # Dict with meta data along with cells in cluster
        clusters.append({
            'cells': cells,
            'size': len(cells),
            'centroid_row': cells[:, 0].mean(),
            'centroid_col': cells[:, 1].mean(),
        })

    return clusters

def score_clusters(
    clusters: list[dict],
    grid: np.ndarray,
    robot_row: int,
    robot_col: int,
    resolution: float,
    alpha: float = 1.0,
    beta: float = 2.0,
    radius_meters: float = 0.5,
    epsilon: float = 1e-6
):

    height, width = grid.shape
    radius_cells = int(radius_meters / resolution) # Physical to grid conversion
    scored = []

    for cluster in clusters:
        # Storing centroid data of cluster
        cr = cluster['centroid_row']
        cc = cluster['centroid_col']

        size = cluster['size']

        # Finding amount of unknown area around cluster 
        r_min = max(0, int(cr) - radius_cells)
        c_min = max(0, int(cc) - radius_cells)
        r_max = min(height, int(cr) + radius_cells + 1)
        c_max = min(width, int(cc) + radius_cells + 1)
        unknown_near = int(np.sum(grid[r_min:r_max, c_min:c_max] == -1))

        # Finding distance between robot and cluster centroid
        dist = np.sqrt((cr - robot_row) ** 2 + (cc - robot_col) ** 2) * resolution

        # Scoring formula
        score = (alpha * size + beta * unknown_near) / (dist + epsilon)

        scored.append({
            **cluster,
            'unknown_near': unknown_near,
            'dist_meters':  dist,
            'score':        score,
        })

    scored.sort(key=lambda x: x['score'], reverse=True) # Sort cluster list by score, highest to lowest
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
        pose = Pose()
        pose.position.x    = wx
        pose.position.y    = wy
        pose.position.z    = 0.0
        pose.orientation.w = 1.0   
        pa.poses.append(pose)

    return pa

def rank_to_rgb(t: float):

    if t < 0.5:
        # green to yellow; increase red, keep green at 1
        r = 2.0 * t
        g = 1.0
    else:
        # yellow to red; decrease green, keep red at 1
        r = 1.0
        g = 1.0 - 2.0 * (t - 0.5)
    return r, g, 0.0

def build_marker_array(scored_clusters, map_msg, stamp):

    ma = MarkerArray()
    n = len(scored_clusters)
    MAX_FRONTIERS = 1000   # offset for cell-cloud marker IDs

    res    = map_msg.info.resolution
    origin = map_msg.info.origin

    for i, cluster in enumerate(scored_clusters):
        t        = i / max(n - 1, 1)
        r, g, b  = rank_to_rgb(t)

        # ---- 1. Centroid sphere ----
        wx, wy = grid_to_world(
            cluster['centroid_row'],
            cluster['centroid_col'],
            origin, res
        )

        centroid_marker                    = Marker()
        centroid_marker.header.frame_id    = 'map'
        centroid_marker.header.stamp       = stamp
        centroid_marker.ns                 = 'frontier_centroids'
        centroid_marker.id                 = i
        centroid_marker.type               = Marker.SPHERE
        centroid_marker.action             = Marker.ADD

        centroid_marker.pose.position.x    = wx
        centroid_marker.pose.position.y    = wy
        centroid_marker.pose.position.z    = 0.05   # slightly above the ground plane
        centroid_marker.pose.orientation.w = 1.0

        centroid_marker.scale.x            = 0.20
        centroid_marker.scale.y            = 0.20
        centroid_marker.scale.z            = 0.20

        centroid_marker.color.r            = r
        centroid_marker.color.g            = g
        centroid_marker.color.b            = b
        centroid_marker.color.a            = 1.0

        centroid_marker.lifetime.sec       = 1
        centroid_marker.lifetime.nanosec   = 0

        ma.markers.append(centroid_marker)

        # ---- 2. Cell cloud (POINTS marker — one marker for all cells) ----
        cloud_marker                    = Marker()
        cloud_marker.header.frame_id    = 'map'
        cloud_marker.header.stamp       = stamp
        cloud_marker.ns                 = 'frontier_cells'
        cloud_marker.id                 = i + MAX_FRONTIERS
        cloud_marker.type               = Marker.POINTS
        cloud_marker.action             = Marker.ADD

        # POINTS markers use scale.x and scale.y as point width/height in meters
        # 0.05 m matches the map resolution so each point fills exactly one cell
        cloud_marker.scale.x            = res
        cloud_marker.scale.y            = res

        cloud_marker.color.r            = r
        cloud_marker.color.g            = g
        cloud_marker.color.b            = b
        cloud_marker.color.a            = 0.6   # slightly transparent so the map shows through

        cloud_marker.lifetime.sec       = 1
        cloud_marker.lifetime.nanosec   = 0

        # pose is identity — each point in the list is in the map frame directly
        cloud_marker.pose.orientation.w = 1.0

        for (row, col) in cluster['cells']:
            cx, cy = grid_to_world(row, col, origin, res)
            p      = Point()
            p.x    = cx
            p.y    = cy
            p.z    = 0.0
            cloud_marker.points.append(p)

        ma.markers.append(cloud_marker)

    return ma


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('min_cluster_size', 10)      # Clustering: Amount of cells in cluster before being filtered
        self.declare_parameter('alpha', 1.0)                # Scoring: Weight of consideration for cluster size
        self.declare_parameter('beta', 2.0)                 # Scoring: Weight of consideration for unknown / distance ratio
        self.declare_parameter('epsilon', 1e-6)             # Scoring: Minimal constant to prevent a division by zero
        self.declare_parameter('radius_meters', 0.5)        # Scoring: Radius from frontier centroid used considered

        # Subscribers
        self.map_sub  = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)

        # Publishers
        self.goals_pub  = self.create_publisher(PoseArray, '/frontier_goals', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/frontier_markers', 10)

        # Robot-Map position coordinates
        self.robot_x = 0.0
        self.robot_y = 0.0

        self.get_logger().info('Frontier Explorer Node started.')

    def pose_callback(self, msg: Odometry): # Store positions from odom to node
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def map_callback(self, msg: OccupancyGrid):
        # Map data comes in 1D array, reformat into 2D array
        grid = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

        # Convert odom position to grid cell position
        robot_col = int((self.robot_x - msg.info.origin.position.x) / msg.info.resolution)
        robot_row = int((self.robot_y - msg.info.origin.position.y) / msg.info.resolution)

        # Read parameters
        min_cluster_size = self.get_parameter('min_cluster_size').value
        alpha = self.get_parameter('alpha').value
        beta = self.get_parameter('beta').value
        radius_meters = self.get_parameter('radius_meters').value
        epsilon = self.get_parameter('epsilon').value

        # frontier pipeline
        frontier_mask = find_frontiers(grid)
        clusters = cluster_frontiers(frontier_mask, min_cluster_size)
        scored = score_clusters(clusters, grid, robot_row, robot_col, msg.info.resolution, alpha, beta, radius_meters, epsilon)

        # Assume nothing returning after scoring means there are no frontiers
        if not scored:
            self.get_logger().info('No frontiers detected.')
            return

        # Publish to new topic
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