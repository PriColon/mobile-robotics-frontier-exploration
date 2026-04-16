import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid

# === BARE BONES === #
class SemanticHazardClassifier(Node):

    def __init__(self):
        super().__init__('semantic_hazard_classifier')

        self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/oakd/stereo/image_raw', self.depth_callback, 10)

        self.semantic_map_pub = self.create_publisher(OccupancyGrid, '/semantic_map', 10)

        self.get_logger().info('Semantic Hazard Classifier Node started.')

    def rgb_callback(self, msg: Image):
        pass

    def depth_callback(self, msg: Image):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = SemanticHazardClassifier()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()