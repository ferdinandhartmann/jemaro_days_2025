import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
import transforms3d
from visualization_msgs.msg import Marker, MarkerArray

from path_planning import dubins  # custom Dubins planner

class DubinsPathPublisher(Node):
    def __init__(self):
        super().__init__('dubins_path_publisher')
        self.publisher_ = self.create_publisher(Path, '/jemaro_path', 10)
        self.marker_publisher_ = self.create_publisher(MarkerArray, '/start_goal_markers', 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/jemaro_map',
            self.map_callback,
            qos_profile=rclpy.qos.QoSProfile(
                depth=10,
                reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
            )
        )

        self.map_data = None
        self.turning_radius = 2.0

        # Start and goal positions (x, y, yaw in radians)
        self.start = (1000.0, 730.0, math.pi / 2)
        self.goal = (1050.0, 750.0, math.pi / 2)

        # Timer to publish markers periodically
        self.marker_timer = self.create_timer(1.0, self.publish_markers)

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Received new map. Computing path...")

        self.compute_and_publish_path()

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

    def publish_markers(self):
        marker_array = MarkerArray()

        start_marker = Marker()
        start_marker.header.frame_id = 'map'
        start_marker.header.stamp = self.get_clock().now().to_msg()
        start_marker.ns = 'start_goal'
        start_marker.id = 0
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = self.start[0]
        start_marker.pose.position.y = self.start[1]
        start_marker.pose.position.z = 0.0
        start_marker.scale.x = 3.0
        start_marker.scale.y = 3.0
        start_marker.scale.z = 3.0
        start_marker.color.a = 1.0
        start_marker.color.r = 0.0
        start_marker.color.g = 1.0
        start_marker.color.b = 0.0

        goal_marker = Marker()
        goal_marker.header.frame_id = 'map'
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.ns = 'start_goal'
        goal_marker.id = 1
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = self.goal[0]
        goal_marker.pose.position.y = self.goal[1]
        goal_marker.pose.position.z = 0.0
        goal_marker.scale.x = 3.0
        goal_marker.scale.y = 3.0
        goal_marker.scale.z = 3.0
        goal_marker.color.a = 1.0
        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.0
        goal_marker.color.b = 0.0

        marker_array.markers.append(start_marker)
        marker_array.markers.append(goal_marker)

        self.marker_publisher_.publish(marker_array)
        self.get_logger().info("Published start and goal markers.")

    def compute_and_publish_path(self):
        start_pt = dubins.Waypoint(self.start[0], self.start[1], math.degrees(self.start[2]))
        goal_pt = dubins.Waypoint(self.goal[0], self.goal[1], math.degrees(self.goal[2]))

        try:
            param = dubins.calcDubinsPath(start_pt, goal_pt, vel=5.0, phi_lim=30)
            traj = dubins.dubins_traj(param, step=0.5)

            if not traj or len(traj) == 0:
                self.get_logger().error("Dubins trajectory is empty. Aborting path generation.")
                return

            self.get_logger().info(f"Generated trajectory with {len(traj)} points.")

            configurations = []
            for i, p in enumerate(traj):
                if i >= len(traj):
                    self.get_logger().error(f"Index {i} is out of bounds for trajectory of size {len(traj)}.")
                    break
                if len(p) < 3:
                    self.get_logger().error(f"Trajectory point {i} is malformed: {p}. Skipping.")
                    continue
                configurations.append((p[0], p[1], p[2]))

            if not self.is_collision_free(configurations):
                self.get_logger().warn("Initial path in collision. Trying left-shifted goal...")
                shifted_goal = self.shift_goal_left(self.goal, distance=1.0)
                shifted_goal_pt = dubins.Waypoint(shifted_goal[0], shifted_goal[1], math.degrees(shifted_goal[2]))

                param = dubins.calcDubinsPath(start_pt, shifted_goal_pt, vel=5.0, phi_lim=30)
                traj = dubins.dubins_traj(param, step=0.5)

                if not traj or len(traj) == 0:
                    self.get_logger().error("Shifted Dubins trajectory is empty. Aborting path generation.")
                    return

                self.get_logger().info(f"Generated shifted trajectory with {len(traj)} points.")

                configurations = []
                for i, p in enumerate(traj):
                    if i >= len(traj):
                        self.get_logger().error(f"Index {i} is out of bounds for shifted trajectory of size {len(traj)}.")
                        break
                    if len(p) < 3:
                        self.get_logger().error(f"Shifted trajectory point {i} is malformed: {p}. Skipping.")
                        continue
                    configurations.append((p[0], p[1], p[2]))

                if not self.is_collision_free(configurations):
                    self.get_logger().error("Shifted path also in collision. Aborting.")
                    return
                else:
                    self.goal = shifted_goal
                    self.get_logger().info("Left-shifted path is collision-free. Using that.")

            self.publish_path(configurations)
            self.publish_markers()

        except IndexError as e:
            self.get_logger().error(f"Index error during Dubins path generation: {e}")
            self.get_logger().debug(f"Trajectory size: {len(traj) if traj else 'N/A'}")
        except Exception as e:
            self.get_logger().error(f"Failed to generate Dubins path: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DubinsPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


### Code which generates a manually calculated straight path

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Path
# from geometry_msgs.msg import PoseStamped
# import math

# class StraightPathPublisher(Node):
#     def __init__(self):
#         super().__init__('straight_path_publisher')
#         self.publisher_ = self.create_publisher(Path, '/path', 10)
#         self.timer = self.create_timer(1.0, self.timer_callback)

#         # Exact start and end poses in 'world' coordinate
#         # self.start = {
#         #     "x": 1.5388960819545028,
#         #     "y": -44.17189175794992,
#         #     "z": 0.05748737734220649,
#         #     "qx": -4.870341654186212e-07,
#         #     "qy": 2.208811513094278e-05,
#         #     "qz": 0.9998483678707415,
#         #     "qw": 0.017413809982094696
#         # }

#         # self.goal = {
#         #     "x": 1.520817118382381,
#         #     "y": 30.988084603474388,
#         #     "z": 0.05748761483841908,
#         #     "qx": -1.3895804518744873e-07,
#         #     "qy": 2.1788112739575307e-05,
#         #     "qz": 0.9999998906502604,
#         #     "qw": 0.00046714529433441895
#         # }

#         # Exact start and end poses in 'map' coordinate
#         self.start = {
#             "x": 1000.0,
#             "y": 730.0,
#             "z": 0.0,
#             "qx": 0.0,
#             "qy": 0.0,
#             "qz": 0.0,
#             "qw": 1.0
#         }

#         self.goal = {
#             "x": 1050.0,
#             "y": 750.0,
#             "z": 0.0,
#             "qx": 0.0,
#             "qy": 0.0,
#             "qz": 0.0,
#             "qw": 1.0
#         }

#     def timer_callback(self):
#         path_msg = Path()
#         path_msg.header.frame_id = 'map'
#         path_msg.header.stamp = self.get_clock().now().to_msg()

#         num_points = 20

#         for i in range(num_points + 1):
#             t = i / num_points
#             pose = PoseStamped()
#             pose.header = path_msg.header

#             # Linear interpolation between start and goal
#             pose.pose.position.x = self.start["x"] + t * (self.goal["x"] - self.start["x"])
#             pose.pose.position.y = self.start["y"] + t * (self.goal["y"] - self.start["y"])
#             pose.pose.position.z = self.start["z"] + t * (self.goal["z"] - self.start["z"])

#             # Simple linear interpolation for quaternion (not normalized — good enough for nearly straight line)
#             pose.pose.orientation.x = self.start["qx"] + t * (self.goal["qx"] - self.start["qx"])
#             pose.pose.orientation.y = self.start["qy"] + t * (self.goal["qy"] - self.start["qy"])
#             pose.pose.orientation.z = self.start["qz"] + t * (self.goal["qz"] - self.start["qz"])
#             pose.pose.orientation.w = self.start["qw"] + t * (self.goal["qw"] - self.start["qw"])

#             path_msg.poses.append(pose)

#         self.publisher_.publish(path_msg)
#         self.get_logger().info('Published straight path with {} poses.'.format(len(path_msg.poses)))

# def main(args=None):
#     rclpy.init(args=args)
#     node = StraightPathPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
