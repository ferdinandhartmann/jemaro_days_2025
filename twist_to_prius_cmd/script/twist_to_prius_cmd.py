#!/usr/bin/env python3

from rclpy.node import Node
from geometry_msgs.msg import Twist
from prius_msgs.msg import Control
from builtin_interfaces.msg import Time
import rclpy

MAX_SPEED = 10.0  # m/s
MAX_STEER_ANGLE = 1.0  # normalized [-1, 1]

class TwistToPriusCmd(Node):
    def __init__(self):
        super().__init__('twist_to_prius_cmd')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.publisher = self.create_publisher(Control, '/prius/control', 10)

    def cmd_vel_callback(self, msg: Twist):
        prius_cmd_msg = Control()

        # Set the header
        now = self.get_clock().now().to_msg()
        prius_cmd_msg.header.stamp = now
        prius_cmd_msg.header.frame_id = "base_link"

        # Determine forward or reverse
        if msg.linear.x > 0:
            prius_cmd_msg.throttle = min(msg.linear.x / MAX_SPEED, 1.0)
            prius_cmd_msg.brake = 0.0
            prius_cmd_msg.shift_gears = Control.FORWARD
        elif msg.linear.x < 0:
            prius_cmd_msg.throttle = min(abs(msg.linear.x) / MAX_SPEED, 1.0)
            prius_cmd_msg.brake = 0.0
            prius_cmd_msg.shift_gears = Control.REVERSE
        else:
            prius_cmd_msg.throttle = 0.0
            prius_cmd_msg.brake = 1.0
            prius_cmd_msg.shift_gears = Control.NEUTRAL

        # Map angular.z to steering
        prius_cmd_msg.steer = max(min(msg.angular.z / MAX_STEER_ANGLE, 1.0), -1.0)

        self.publisher.publish(prius_cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToPriusCmd()
    rclpy.spin(node)
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()
 
 
