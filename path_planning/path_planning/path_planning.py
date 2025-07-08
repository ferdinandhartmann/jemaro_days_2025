### Code which generates a straight path with Dubins  
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import easydubins
import numpy as np
import math
import transforms3d


class DubinsPathPublisher(Node):
    def __init__(self):
        super().__init__('dubins_path_publisher')

        self.publisher_ = self.create_publisher(Path, '/path', 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.map_data = None
        self.turning_radius = 2.0

        # Start and goal: (x, y, yaw)
        self.start = (1000, 730, math.pi / 2)
        self.goal = (1050, 750, math.pi / 2)

    def map_callback(self, msg):
        self.map_data = msg

    def is_collision_free(self, path_points):
        if self.map_data is None:
            self.get_logger().warn("Map not received yet.")
            return True

        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin.position
        data = np.array(self.map_data.data).reshape((height, width))

        for x, y, _ in path_points:
            mx = int((x - origin.x) / resolution)
            my = int((y - origin.y) / resolution)
            if 0 <= mx < width and 0 <= my < height:
                if data[my, mx] > 50:
                    return False
        return True

    def shift_goal_left(self, goal, distance=1.0):
        x, y, yaw = goal
        # Shift left is perpendicular to the heading
        left_x = x - distance * math.sin(yaw)
        left_y = y + distance * math.cos(yaw)
        return (left_x, left_y, yaw)

    def publish_path(self, configurations):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for conf in configurations:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = conf[0]
            pose.pose.position.y = conf[1]
            pose.pose.position.z = 0.0

            yaw = conf[2]
            quat = transforms3d.quaternions.axangle2quat([0, 0, 1], yaw)
            pose.pose.orientation.x = quat[1]
            pose.pose.orientation.y = quat[2]
            pose.pose.orientation.z = quat[3]
            pose.pose.orientation.w = quat[0]

            path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)
        self.get_logger().info(f"Published Dubins path with {len(path_msg.poses)} poses.")

    def timer_callback(self):
        path = easydubins.path(self.start, self.goal, self.turning_radius)
        xs, ys, yaws = path.sample_many(0.5)
        configurations = list(zip(xs, ys, yaws))

        if not self.is_collision_free(configurations):
            self.get_logger().warn("Initial path in collision. Trying to shift goal to the left.")
            shifted_goal = self.shift_goal_left(self.goal, distance=1.0)
            path = easydubins.path(self.start, shifted_goal, self.turning_radius)
            xs, ys, yaws = path.sample_many(0.5)
            configurations = list(zip(xs, ys, yaws))

            if not self.is_collision_free(configurations):
                self.get_logger().error("Shifted path also in collision. Aborting.")
                return
            else:
                self.goal = shifted_goal
                self.get_logger().info("Successfully found left-shifted collision-free path.")

        self.publish_path(configurations)


def main(args=None):
    rclpy.init(args=args)
    node = DubinsPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


### Code which generates a manually calculated straight path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class StraightPathPublisher(Node):
    def __init__(self):
        super().__init__('straight_path_publisher')
        self.publisher_ = self.create_publisher(Path, '/path', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Exact start and end poses in 'world' coordinate
        # self.start = {
        #     "x": 1.5388960819545028,
        #     "y": -44.17189175794992,
        #     "z": 0.05748737734220649,
        #     "qx": -4.870341654186212e-07,
        #     "qy": 2.208811513094278e-05,
        #     "qz": 0.9998483678707415,
        #     "qw": 0.017413809982094696
        # }

        # self.goal = {
        #     "x": 1.520817118382381,
        #     "y": 30.988084603474388,
        #     "z": 0.05748761483841908,
        #     "qx": -1.3895804518744873e-07,
        #     "qy": 2.1788112739575307e-05,
        #     "qz": 0.9999998906502604,
        #     "qw": 0.00046714529433441895
        # }

        # Exact start and end poses in 'map' coordinate
        self.start = {
            "x": 1000.0,
            "y": 730.0,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0
        }

        self.goal = {
            "x": 1050.0,
            "y": 750.0,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0
        }

    def timer_callback(self):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        num_points = 20

        for i in range(num_points + 1):
            t = i / num_points
            pose = PoseStamped()
            pose.header = path_msg.header

            # Linear interpolation between start and goal
            pose.pose.position.x = self.start["x"] + t * (self.goal["x"] - self.start["x"])
            pose.pose.position.y = self.start["y"] + t * (self.goal["y"] - self.start["y"])
            pose.pose.position.z = self.start["z"] + t * (self.goal["z"] - self.start["z"])

            # Simple linear interpolation for quaternion (not normalized — good enough for nearly straight line)
            pose.pose.orientation.x = self.start["qx"] + t * (self.goal["qx"] - self.start["qx"])
            pose.pose.orientation.y = self.start["qy"] + t * (self.goal["qy"] - self.start["qy"])
            pose.pose.orientation.z = self.start["qz"] + t * (self.goal["qz"] - self.start["qz"])
            pose.pose.orientation.w = self.start["qw"] + t * (self.goal["qw"] - self.start["qw"])

            path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)
        self.get_logger().info('Published straight path with {} poses.'.format(len(path_msg.poses)))

def main(args=None):
    rclpy.init(args=args)
    node = StraightPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
