import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
# import transforms3d
from visualization_msgs.msg import Marker, MarkerArray

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import rclpy.qos

class DubinsPathPublisher(Node):
    def __init__(self):
        super().__init__('dubins_path_publisher')
        # self.publisher_ = self.create_publisher(Path, '/jemaro_path', 10) 
        self.publisher_ = self.create_publisher(Path, '/ZOE3/path_follower/setPath', 10) 
        self.marker_publisher_ = self.create_publisher(MarkerArray, '/start_goal_markers', 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/jemaro_map',
            self.map_callback,
            qos_profile=rclpy.qos.QoSProfile(
                depth=10,
                reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            )
        )
        self.point_marker_pub = self.create_publisher(MarkerArray, '/path_points', 10)

        self.marker_timer = self.create_timer(1.0, self.publish_markers)
        self.path_timer = self.create_timer(4.5, self.compute_and_publish_path)

        self.map_data = None
        self.turning_radius = 2.0

        # Start and goal positions (x, y, yaw in radians)
        self.start = (1097.2, 776.28, math.pi / 2)
        self.goal = (1012.74, 739.0, math.pi / 2)

        self.get_logger().info("Started Path Planning Node")


    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Received new map.")

        # self.compute_and_publish_path()


    def adjust_path_for_obstacles(
        self,
        points,
        shift_distance: float = 0.1, # distance to shift the points each iteration
        window_size_meters: float = 0.0, 
        safety_distance: float = 0.8,
        shift_direction: int = 1
    ):
        """
        Slide any window of points that collide with obstacles laterally by shift_distance
        (to the left if shift_direction=+1, to the right if -1), checking a safety buffer
        around each shifted point. Skips ahead past the window after shifting.
        """
        if self.map_data is None:
            return points

        # Map metadata
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin.position
        grid = np.array(self.map_data.data).reshape((height, width))

        # Precompute path direction
        dx = self.goal[0] - self.start[0]
        dy = self.goal[1] - self.start[1]
        path_angle = math.atan2(dy, dx)

        # Convert meters to cells
        safety_cells = int(safety_distance / resolution)
        window_size = int(window_size_meters / resolution)

        adjusted = []
        i = 0
        max_attempts = 500

        while i < len(points):
            x, y, yaw = points[i]
            mx = int((x - origin.x) / resolution)
            my = int((y - origin.y) / resolution)

            # Out‐of‐bounds → keep as is
            if not (0 <= mx < width and 0 <= my < height):
                adjusted.append([x, y, yaw])
                i += 1
                continue

            # Check collision + safety buffer
            is_obstacle = False
            left_occupied = 0
            right_occupied = 0
            total_cells = 0

            for ox in range(-safety_cells, safety_cells + 1):
                for oy in range(-safety_cells, safety_cells + 1):
                    cx = mx + ox
                    cy = my + oy
                    if 0 <= cx < width and 0 <= cy < height:
                        if grid[cy, cx] > 80:
                            # Determine if the cell is on the left or right of the path
                            relative_x = (cx * resolution + origin.x) - x
                            relative_y = (cy * resolution + origin.y) - y
                            cross_product = relative_x * math.sin(path_angle) - relative_y * math.cos(path_angle)
                            if cross_product > 0:
                                left_occupied += 1
                            else:
                                right_occupied += 1
                        total_cells += 1

            is_obstacle = left_occupied + right_occupied > 0

            # Determine shift direction based on occupancy
            if left_occupied > right_occupied:
                shift_direction = 1  # Shift to the right
                
            else:
                shift_direction = -1  # Shift to the left

            if is_obstacle:
                self.get_logger().info(f"Obstacle at point {i}, shifting window ±{window_size} cells")

                # Define the block of points to shift
                start_idx = max(0, i - window_size)
                end_idx = min(len(points), i + window_size + 1)

                # Shift each point in that block laterally
                for j in range(start_idx, end_idx):
                    px, py, pyaw = points[j]
                    shifted = False

                    for attempt in range(max_attempts):
                        # lateral shift
                        sx = px - shift_direction * shift_distance * math.sin(path_angle)
                        sy = py + shift_direction * shift_distance * math.cos(path_angle)

                        mxs = int((sx - origin.x) / resolution)
                        mys = int((sy - origin.y) / resolution)

                        # safety check around shifted cell
                        safe = True
                        for sox in range(-safety_cells, safety_cells + 1):
                            for soy in range(-safety_cells, safety_cells + 1):
                                cxx = mxs + sox
                                cyy = mys + soy
                                if 0 <= cxx < width and 0 <= cyy < height and grid[cyy, cxx] > 80:
                                    safe = False
                                    break
                            if not safe:
                                break

                        if safe:
                            adjusted.append([sx, sy, pyaw])
                            shifted = True
                            break
                        else:
                            # adjusted.append([sx, sy, pyaw])
                            # Try shifting further on next iteration
                            px, py = sx, sy
    
                        # Publish the path after each iteration for viz
                        # self.publish_path(adjusted)

                    if not shifted:
                        self.get_logger().error(f"Could not safely shift point {j} after {max_attempts} tries")
                        adjusted.append([px, py, pyaw])


                # Skip past the window
                i = end_idx

            else:
                # No obstacle here → keep original
                adjusted.append([x, y, yaw])
                i += 1

        return adjusted


    def smooth_path(self, points, kernel_size=50):
        import scipy.ndimage

        if len(points) < kernel_size:
            return points

        points_np = np.array(points)
        smoothed = np.copy(points_np)

        for i in range(3):  # x, y, yaw
            smoothed[:, i] = scipy.ndimage.uniform_filter1d(points_np[:, i], size=kernel_size, mode='nearest')

        return smoothed.tolist()

    def fill_gaps(self, pts, max_dist=1.0):
        """
        Given a list of [x,y,yaw], insert extra points between any two
        whose Euclidean gap > max_dist.
        """
        filled = []
        for (x0,y0,yaw0), (x1,y1,yaw1) in zip(pts, pts[1:]):
            filled.append([x0,y0,yaw0])
            dx = x1 - x0
            dy = y1 - y0
            dist = math.hypot(dx,dy)
            if dist > max_dist:
                n = int(math.ceil(dist / max_dist))
                for k in range(1, n):
                    t = k / n
                    filled.append([
                        x0 + t*dx,
                        y0 + t*dy,
                        yaw0 + t*((yaw1-yaw0+math.pi)%(2*math.pi)-math.pi)  # shortest‐angle interp
                    ])
        # append last
        filled.append(pts[-1])
        return filled

    def compute_and_publish_path(self):
        if self.map_data is None:
            self.get_logger().warn("Map data is not available yet. Skipping path computation.")
            return

        num_points = 100
        x0, y0, yaw0 = self.start
        x1, y1, yaw1 = self.goal

        points = []
        for i in range(num_points + 1):
            t = i / num_points
            x = x0 + t * (x1 - x0)
            y = y0 + t * (y1 - y0)
            yaw = yaw0 + t * (yaw1 - yaw0)
            points.append([x, y, yaw])

        # Adjust points if collision is detected
        adjusted = self.adjust_path_for_obstacles(points)

        # bridge gaps:
        resolution = self.map_data.info.resolution
        step = resolution * 1.5
        bridged = self.fill_gaps(adjusted, max_dist=step)

        # now publish `bridged` instead of `adjusted`
        smoothed = self.smooth_path(bridged)

        self.publish_path(smoothed)

    
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

            # yaw = conf[2]
            # quat = self.yaw_to_quaternion(yaw)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.publisher_.publish(path_msg)

        self.publish_point_markers(configurations)

        self.get_logger().info(f"Published Dubins path with {len(path_msg.poses)} poses.")

    def publish_point_markers(self, pts):
        ma = MarkerArray()

        # Clear existing markers by publishing an empty array
        clear_ma = MarkerArray()
        self.point_marker_pub.publish(clear_ma)

        for i, (x, y, yaw) in enumerate(pts):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'path_points'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.1
            m.scale.x = 0.5
            m.scale.y = 0.5
            m.scale.z = 0.5
            m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
            ma.markers.append(m)

        self.point_marker_pub.publish(ma)


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


def main(args=None):
    rclpy.init(args=args)
    node = DubinsPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

