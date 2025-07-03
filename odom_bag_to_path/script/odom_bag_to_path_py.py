#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import rosbag2_py
import numpy as np
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import deserialize_message

class OdomBagToPath(Node):

    def __init__(self):
        super().__init__('odom_bag_to_path_py')
        
        self.declare_parameter('dataset', 'path')
        dataset = self.get_parameter('dataset').get_parameter_value().string_value
        self.declare_parameter('topic', 'odom')
        odom_topic = self.get_parameter('topic').get_parameter_value().string_value
        self.declare_parameter('frequency', 10)
        frequency = self.get_parameter('frequency').get_parameter_value().integer_value
        
        self.reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(
            uri=dataset,
            storage_id='sqlite3')
            
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.reader.open(storage_options, converter_options)

        self.path = Path()
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.declare_parameter('frame_id', 'world')
        self.path.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        downsample_i = 0
        self.declare_parameter('downsample_n', 100)
        downsample_n = self.get_parameter('downsample_n').get_parameter_value().integer_value
        while self.reader.has_next():
            topic, data, timestamp = self.reader.read_next()
            if topic == odom_topic:
                downsample_i += 1
                if downsample_i >= downsample_n :
                    downsample_i = 0
                    msg = deserialize_message(data, Odometry)
                    pose = PoseStamped()
                    pose.header = msg.header
                    pose.pose = msg.pose.pose
                    self.path.poses.append(pose)
        print('Path loaded !')
 
        self.path_publisher = self.create_publisher(Path, 'path', 10)
        
        self.timer = self.create_timer(frequency, self.timer_callback)
        
    def timer_callback(self):
        self.path_publisher.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    odom_bag_to_path_py = OdomBagToPath()
    rclpy.spin(odom_bag_to_path_py)
    simulation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
