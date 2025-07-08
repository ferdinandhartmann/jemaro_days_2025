
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

        # Exact start and end poses
        self.start = {
            "x": 1.5388960819545028,
            "y": -44.17189175794992,
            "z": 0.05748737734220649,
            "qx": -4.870341654186212e-07,
            "qy": 2.208811513094278e-05,
            "qz": 0.9998483678707415,
            "qw": 0.017413809982094696
        }

        self.goal = {
            "x": 1.520817118382381,
            "y": 30.988084603474388,
            "z": 0.05748761483841908,
            "qx": -1.3895804518744873e-07,
            "qy": 2.1788112739575307e-05,
            "qz": 0.9999998906502604,
            "qw": 0.00046714529433441895
        }

    def timer_callback(self):
        path_msg = Path()
        path_msg.header.frame_id = 'world'
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
