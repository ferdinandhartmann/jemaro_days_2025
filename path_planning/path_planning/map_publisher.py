import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

class StaticMapPublisher(Node):
    def __init__(self):
        super().__init__('static_map_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/test_map', 10)
        self.declare_parameter('publish_once', True)

        self.map_msg = self.initialize_map()
        self.timer = self.create_timer(1.0, self.publish_map)

    def initialize_map(self):
        width = 6000
        height = 6000
        resolution = 0.25  # meters per cell

        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'map'
        map_msg.info.resolution = resolution
        map_msg.info.width = width
        map_msg.info.height = height

        origin = Pose()
        origin.position.x = 0.0
        origin.position.y = 0.0
        origin.position.z = 0.0
        origin.orientation.w = 1.0
        map_msg.info.origin = origin

        # Initialize all cells to free (0)
        map_msg.data = [0] * (width * height)
        return map_msg

    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.map_msg)
        self.get_logger().info('Published static map.')

        # Optionally stop publishing after first time
        if self.get_parameter('publish_once').get_parameter_value().bool_value:
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = StaticMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
