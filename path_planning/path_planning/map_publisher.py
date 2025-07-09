import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import math

class StaticMapPublisher(Node):
    def __init__(self):
        super().__init__('static_map_publisher')
        qos = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        self.publisher_ = self.create_publisher(OccupancyGrid, '/jemaro_map', qos)

        self.map_msg = self.initialize_map()
        self.timer = self.create_timer(0.7, self.publish_map)

        self.square_active = False  # Track if the square is active
        self.publish_count = 0  # Count the number of times the map is published

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
    
    def toggle_square(self):
        width = self.map_msg.info.width
        height = self.map_msg.info.height

        # Square parameters
        square_size_x = 15
        square_size_y = 20
        resolution = self.map_msg.info.resolution

        # Start and goal in meters
        start = (1000.0, 730.0)
        goal = (1050.0, 750.0)

        # Convert to grid cells
        start_cell = (int(start[0] / resolution), int(start[1] / resolution))
        goal_cell = (int(goal[0] / resolution), int(goal[1] / resolution))

        # Direction unit vector
        dx = goal_cell[0] - start_cell[0]
        dy = goal_cell[1] - start_cell[1]
        length = math.sqrt(dx**2 + dy**2)
        dx /= length
        dy /= length

        # Positions for two separated squares (e.g. 40 and 70 cells from start)
        distances = [10, 50]

        for dist in distances:
            square_start_x = int(start_cell[0] + dx * dist)
            square_start_y = int(start_cell[1] + dy * dist)

            for y in range(square_start_y, square_start_y + square_size_y):
                for x in range(square_start_x, square_start_x + square_size_x):
                    if 0 <= x < width and 0 <= y < height:
                        index = y * width + x
                        self.map_msg.data[index] = 0 if self.square_active else 100

        self.square_active = not self.square_active



    # def toggle_square(self):
    #     width = self.map_msg.info.width
    #     height = self.map_msg.info.height

    #     # Define square parameters
    #     square_size_x = 15  # 150 cells in x direction
    #     square_size_y = 20  # 300 cells in y direction
    #     x_distance = 40  # Distance along the line between start and goal

    #     # Start and goal positions in map frame meters
    #     start = (1000.0, 730.0)
    #     goal = (1050.0, 750.0)

    #     # Convert start and goal positions to grid cells
    #     resolution = self.map_msg.info.resolution
    #     start_cell = (int(start[0] / resolution), int(start[1] / resolution))
    #     goal_cell = (int(goal[0] / resolution), int(goal[1] / resolution))

    #     # Calculate the direction vector and normalize it
    #     direction_x = goal_cell[0] - start_cell[0]
    #     direction_y = goal_cell[1] - start_cell[1]
    #     length = math.sqrt(direction_x**2 + direction_y**2)
    #     direction_x /= length
    #     direction_y /= length

    #     # Calculate the square's starting position along the line
    #     square_start_x = int(start_cell[0] + direction_x * x_distance)
    #     square_start_y = int(start_cell[1] + direction_y * x_distance)

    #     for y in range(square_start_y, square_start_y + square_size_y):
    #         for x in range(square_start_x, square_start_x + square_size_x):
    #             if 0 <= x < width and 0 <= y < height:  # Ensure within bounds
    #                 index = y * width + x
    #                 if self.square_active:
    #                     self.map_msg.data[index] = 0  # Remove square (set to free)
    #                 else:
    #                     self.map_msg.data[index] = 100  # Add square (set to occupied)

    #     self.square_active = not self.square_active  # Toggle square state


    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()

        # Toggle square every two publishes
        if self.publish_count % 2 == 0:
            self.toggle_square()

        self.publisher_.publish(self.map_msg)
        self.get_logger().info('Published static map.')

        self.publish_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = StaticMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
