import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from irobot_create_msgs.msg import HazardDetectionVector

# === BARE BONES === #
class BehaviorCoordinator(Node):

    def __init__(self):
        super().__init__('behavior_coordinator')

        self.create_subscription(PoseArray,            '/frontier_goals',   self.frontier_callback, 10)
        self.create_subscription(OccupancyGrid,        '/semantic_map',     self.semantic_callback, 10)
        self.create_subscription(BatteryState,         '/battery_state',    self.battery_callback,  10)
        self.create_subscription(HazardDetectionVector,'/hazard_detection', self.hazard_callback,   10)
        self.create_subscription(Bool,                 '/exploration/stop', self.stop_callback,     10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Behavior Coordinator Node started.')

    def frontier_callback(self, msg: PoseArray):
        pass

    def semantic_callback(self, msg: OccupancyGrid):
        pass

    def battery_callback(self, msg: BatteryState):
        pass

    def hazard_callback(self, msg: HazardDetectionVector):
        pass

    def stop_callback(self, msg: Bool):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorCoordinator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()