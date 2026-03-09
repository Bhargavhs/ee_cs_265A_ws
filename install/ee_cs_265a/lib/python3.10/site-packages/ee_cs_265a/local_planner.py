import math
import random
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
import numpy as np


class LocalPlanner(Node):
    """RRT-based local planner for dynamic obstacle avoidance.

    Checks the global path for obstacles detected by lidar.
    When the path is blocked, runs RRT to find a local detour.
    When clear, passes the global path through unchanged.
    """

    def __init__(self):
        super().__init__('local_planner')

        # Parameters
        self.declare_parameter('scan_topic', '/red/scan')
        self.declare_parameter('odom_topic', '/red/odometry')
        self.declare_parameter('obstacle_radius', 0.4)       # inflate detected obstacles
        self.declare_parameter('path_check_distance', 4.0)    # how far ahead to check
        self.declare_parameter('path_block_threshold', 0.6)   # obstacle closer than this blocks
        self.declare_parameter('rrt_step_size', 0.3)
        self.declare_parameter('rrt_max_iter', 500)
        self.declare_parameter('rrt_goal_bias', 0.3)
        self.declare_parameter('rrt_search_radius', 5.0)
        self.declare_parameter('replan_rate', 5.0)
        self.declare_parameter('rejoin_distance', 2.0)        # how far past obstacle to rejoin

        self.obstacle_radius = self.get_parameter('obstacle_radius').value
        self.path_check_dist = self.get_parameter('path_check_distance').value
        self.block_thresh = self.get_parameter('path_block_threshold').value
        self.rrt_step = self.get_parameter('rrt_step_size').value
        self.rrt_max_iter = int(self.get_parameter('rrt_max_iter').value)
        self.rrt_goal_bias = self.get_parameter('rrt_goal_bias').value
        self.rrt_radius = self.get_parameter('rrt_search_radius').value
        self.rejoin_dist = self.get_parameter('rejoin_distance').value

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State
        self.global_path = None
        self.scan = None
        self.inflated_map = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.map_width = None
        self.map_height = None
        self.obstacle_points = []  # dynamic obstacles in map frame

        # Subscriptions
        scan_topic = self.get_parameter('scan_topic').value
        self.create_subscription(Path, '/global_path', self.global_path_cb, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_cb, 10)
        self.create_subscription(OccupancyGrid, '/inflated_map', self.map_cb, 10)

        # Publisher
        self.local_path_pub = self.create_publisher(Path, '/local_path', 10)

        # Timer
        rate = self.get_parameter('replan_rate').value
        self.timer = self.create_timer(1.0 / rate, self.replan)

        self.get_logger().info('Local planner ready')

    def global_path_cb(self, msg):
        self.global_path = msg

    def scan_cb(self, msg):
        self.scan = msg

    def map_cb(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.inflated_map = np.array(msg.data, dtype=np.int8).reshape(
            (self.map_height, self.map_width)
        )

    def get_robot_pose(self):
        """Get robot pose in map frame."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return x, y, yaw
        except Exception:
            return None

    def scan_to_obstacles(self, rx, ry, ryaw):
        """Convert lidar scan to obstacle points in map frame."""
        if self.scan is None:
            return []

        obstacles = []
        angle = self.scan.angle_min
        for r in self.scan.ranges:
            if self.scan.range_min < r < self.scan.range_max:
                # Point in robot frame
                px = r * math.cos(angle)
                py = r * math.sin(angle)
                # Rotate to map frame
                mx = rx + math.cos(ryaw) * px - math.sin(ryaw) * py
                my = ry + math.sin(ryaw) * px + math.cos(ryaw) * py
                obstacles.append((mx, my))
            angle += self.scan.angle_increment

        return obstacles

    def is_static_obstacle(self, x, y):
        """Check if point is a wall/static obstacle on the inflated map."""
        if self.inflated_map is None:
            return False
        gx = int((x - self.map_origin_x) / self.map_resolution)
        gy = int((y - self.map_origin_y) / self.map_resolution)
        if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
            return self.inflated_map[gy, gx] != 0
        return True  # out of bounds = obstacle

    def find_dynamic_obstacles(self, scan_points):
        """Filter scan points to find only dynamic obstacles (not walls)."""
        dynamic = []
        for x, y in scan_points:
            if not self.is_static_obstacle(x, y):
                dynamic.append((x, y))
        return dynamic

    def is_path_blocked(self, rx, ry, path_poses, dynamic_obs):
        """Check if any dynamic obstacles block the path ahead."""
        if not dynamic_obs:
            return False, -1

        for i, pose in enumerate(path_poses):
            px = pose.pose.position.x
            py = pose.pose.position.y

            # Only check ahead within path_check_distance
            d_to_robot = math.sqrt((px - rx) ** 2 + (py - ry) ** 2)
            if d_to_robot > self.path_check_dist:
                break

            for ox, oy in dynamic_obs:
                d = math.sqrt((px - ox) ** 2 + (py - oy) ** 2)
                if d < self.block_thresh:
                    return True, i

        return False, -1

    def is_collision_free(self, x, y, dynamic_obs):
        """Check point against static map + dynamic obstacles."""
        if self.is_static_obstacle(x, y):
            return False
        for ox, oy in dynamic_obs:
            if math.sqrt((x - ox) ** 2 + (y - oy) ** 2) < self.obstacle_radius:
                return False
        return True

    def is_edge_free(self, x1, y1, x2, y2, dynamic_obs):
        """Check line segment for collisions."""
        dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        steps = max(int(dist / 0.1), 2)
        for i in range(steps + 1):
            t = i / steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            if not self.is_collision_free(x, y, dynamic_obs):
                return False
        return True

    def rrt(self, start, goal, dynamic_obs):
        """Run RRT from start to goal, avoiding static + dynamic obstacles."""
        sx, sy = start
        gx, gy = goal

        if not self.is_collision_free(sx, sy, dynamic_obs):
            return None
        if not self.is_collision_free(gx, gy, dynamic_obs):
            return None

        nodes = [(sx, sy)]
        parents = {0: -1}

        for _ in range(self.rrt_max_iter):
            # Sample with goal bias
            if random.random() < self.rrt_goal_bias:
                rx, ry = gx, gy
            else:
                rx = sx + random.uniform(-self.rrt_radius, self.rrt_radius)
                ry = sy + random.uniform(-self.rrt_radius, self.rrt_radius)

            # Find nearest node
            min_d = float('inf')
            nearest_idx = 0
            for i, (nx, ny) in enumerate(nodes):
                d = (nx - rx) ** 2 + (ny - ry) ** 2
                if d < min_d:
                    min_d = d
                    nearest_idx = i

            nx, ny = nodes[nearest_idx]
            dx = rx - nx
            dy = ry - ny
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < 0.01:
                continue

            # Step toward sample
            step = min(self.rrt_step, dist)
            new_x = nx + (dx / dist) * step
            new_y = ny + (dy / dist) * step

            if not self.is_edge_free(nx, ny, new_x, new_y, dynamic_obs):
                continue

            new_idx = len(nodes)
            nodes.append((new_x, new_y))
            parents[new_idx] = nearest_idx

            # Check if we reached goal
            if math.sqrt((new_x - gx) ** 2 + (new_y - gy) ** 2) < self.rrt_step:
                if self.is_edge_free(new_x, new_y, gx, gy, dynamic_obs):
                    goal_idx = len(nodes)
                    nodes.append((gx, gy))
                    parents[goal_idx] = new_idx

                    # Reconstruct path
                    path = []
                    idx = goal_idx
                    while idx != -1:
                        path.append(nodes[idx])
                        idx = parents[idx]
                    path.reverse()
                    return path

        return None  # failed

    def find_closest_idx(self, rx, ry, poses):
        """Find closest pose index on path."""
        min_d = float('inf')
        idx = 0
        for i, p in enumerate(poses):
            dx = p.pose.position.x - rx
            dy = p.pose.position.y - ry
            d = dx * dx + dy * dy
            if d < min_d:
                min_d = d
                idx = i
        return idx

    def replan(self):
        if self.global_path is None:
            return

        pose = self.get_robot_pose()
        if pose is None:
            # Pass through global path if we can't localize
            self.local_path_pub.publish(self.global_path)
            return

        rx, ry, ryaw = pose
        poses = self.global_path.poses

        # Convert scan to map-frame obstacles
        scan_points = self.scan_to_obstacles(rx, ry, ryaw)
        dynamic_obs = self.find_dynamic_obstacles(scan_points)
        self.obstacle_points = dynamic_obs

        # Find closest point on global path
        closest_idx = self.find_closest_idx(rx, ry, poses)

        # Check ahead on global path
        ahead_poses = poses[closest_idx:]
        blocked, block_rel_idx = self.is_path_blocked(rx, ry, ahead_poses, dynamic_obs)

        if not blocked:
            # Path is clear — pass through global path
            self.local_path_pub.publish(self.global_path)
            return

        block_idx = closest_idx + block_rel_idx
        self.get_logger().info(
            f'Path blocked at index {block_idx}, running RRT...',
            throttle_duration_sec=1.0
        )

        # Find rejoin point past the obstacle
        rejoin_idx = block_idx
        bx = poses[block_idx].pose.position.x
        by = poses[block_idx].pose.position.y
        for i in range(block_idx, len(poses)):
            px = poses[i].pose.position.x
            py = poses[i].pose.position.y
            if math.sqrt((px - bx) ** 2 + (py - by) ** 2) > self.rejoin_dist:
                rejoin_idx = i
                break
        else:
            rejoin_idx = len(poses) - 1

        gx = poses[rejoin_idx].pose.position.x
        gy = poses[rejoin_idx].pose.position.y

        # Run RRT
        rrt_path = self.rrt((rx, ry), (gx, gy), dynamic_obs)

        if rrt_path is None:
            self.get_logger().warn(
                'RRT failed, using global path', throttle_duration_sec=1.0
            )
            self.local_path_pub.publish(self.global_path)
            return

        # Build local path: RRT detour + remaining global path
        local_msg = Path()
        local_msg.header.frame_id = 'map'
        local_msg.header.stamp = self.get_clock().now().to_msg()

        # RRT detour
        for x, y in rrt_path:
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.w = 1.0
            local_msg.poses.append(p)

        # Append remaining global path after rejoin
        for p in poses[rejoin_idx + 1:]:
            local_msg.poses.append(p)

        self.local_path_pub.publish(local_msg)
        self.get_logger().info(
            f'Local detour: {len(rrt_path)} RRT pts + '
            f'{len(poses) - rejoin_idx - 1} global pts',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
