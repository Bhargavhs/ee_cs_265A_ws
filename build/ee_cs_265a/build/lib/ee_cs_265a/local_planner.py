import math
import random
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import tf2_ros
import numpy as np


class LocalPlanner(Node):
    """RRT*-based local planner for dynamic obstacle avoidance.

    Checks the global path for obstacles detected by lidar.
    When the path is blocked, runs RRT* to find an optimized local detour.
    When clear, passes the global path through unchanged.
    Publishes the RRT* tree on /rrt_tree for visualization.
    """

    def __init__(self):
        super().__init__('local_planner')

        # Parameters
        self.declare_parameter('scan_topic', '/red/scan')
        self.declare_parameter('obstacle_radius', 0.4)
        self.declare_parameter('path_check_distance', 4.0)
        self.declare_parameter('path_block_threshold', 0.6)
        self.declare_parameter('rrt_step_size', 0.3)
        self.declare_parameter('rrt_max_iter', 500)
        self.declare_parameter('rrt_goal_bias', 0.3)
        self.declare_parameter('rrt_search_radius', 5.0)
        self.declare_parameter('rrt_rewire_radius', 1.5)
        self.declare_parameter('replan_rate', 5.0)
        self.declare_parameter('rejoin_distance', 2.0)

        self.obstacle_radius = self.get_parameter('obstacle_radius').value
        self.path_check_dist = self.get_parameter('path_check_distance').value
        self.block_thresh = self.get_parameter('path_block_threshold').value
        self.rrt_step = self.get_parameter('rrt_step_size').value
        self.rrt_max_iter = int(self.get_parameter('rrt_max_iter').value)
        self.rrt_goal_bias = self.get_parameter('rrt_goal_bias').value
        self.rrt_radius = self.get_parameter('rrt_search_radius').value
        self.rewire_radius = self.get_parameter('rrt_rewire_radius').value
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

        # Subscriptions
        scan_topic = self.get_parameter('scan_topic').value
        self.create_subscription(Path, '/global_path', self.global_path_cb, 10)
        self.create_subscription(LaserScan, scan_topic, self.scan_cb, 10)
        self.create_subscription(OccupancyGrid, '/inflated_map', self.map_cb, 10)

        # Publishers
        self.local_path_pub = self.create_publisher(Path, '/local_path', 10)
        self.rrt_tree_pub = self.create_publisher(Marker, '/rrt_tree', 10)
        self.rrt_path_pub = self.create_publisher(Marker, '/rrt_path', 10)

        # Timer
        rate = self.get_parameter('replan_rate').value
        self.timer = self.create_timer(1.0 / rate, self.replan)

        self.get_logger().info('Local planner (RRT*) ready')

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
        if self.scan is None:
            return []

        obstacles = []
        angle = self.scan.angle_min
        for r in self.scan.ranges:
            if self.scan.range_min < r < self.scan.range_max:
                px = r * math.cos(angle)
                py = r * math.sin(angle)
                mx = rx + math.cos(ryaw) * px - math.sin(ryaw) * py
                my = ry + math.sin(ryaw) * px + math.cos(ryaw) * py
                obstacles.append((mx, my))
            angle += self.scan.angle_increment

        return obstacles

    def is_static_obstacle(self, x, y):
        if self.inflated_map is None:
            return False
        gx = int((x - self.map_origin_x) / self.map_resolution)
        gy = int((y - self.map_origin_y) / self.map_resolution)
        if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
            return self.inflated_map[gy, gx] != 0
        return True

    def find_dynamic_obstacles(self, scan_points):
        if self.inflated_map is None:
            # Without map, can't distinguish static from dynamic — return empty
            return []
        dynamic = []
        for x, y in scan_points:
            if not self.is_static_obstacle(x, y):
                dynamic.append((x, y))
        return dynamic

    def is_path_blocked(self, rx, ry, path_poses, dynamic_obs):
        if not dynamic_obs:
            return False, -1

        for i, pose in enumerate(path_poses):
            px = pose.pose.position.x
            py = pose.pose.position.y

            d_to_robot = math.sqrt((px - rx) ** 2 + (py - ry) ** 2)
            if d_to_robot > self.path_check_dist:
                break

            for ox, oy in dynamic_obs:
                d = math.sqrt((px - ox) ** 2 + (py - oy) ** 2)
                if d < self.block_thresh:
                    return True, i

        return False, -1

    def is_collision_free(self, x, y, dynamic_obs):
        if self.is_static_obstacle(x, y):
            return False
        for ox, oy in dynamic_obs:
            if math.sqrt((x - ox) ** 2 + (y - oy) ** 2) < self.obstacle_radius:
                return False
        return True

    def is_edge_free(self, x1, y1, x2, y2, dynamic_obs):
        dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        steps = max(int(dist / 0.1), 2)
        for i in range(steps + 1):
            t = i / steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            if not self.is_collision_free(x, y, dynamic_obs):
                return False
        return True

    def _dist(self, x1, y1, x2, y2):
        return math.hypot(x2 - x1, y2 - y1)

    def rrt_star(self, start, goal, dynamic_obs):
        """Run RRT* from start to goal, avoiding static + dynamic obstacles.

        Returns (path, nodes, parents) where path is the final waypoint list,
        and nodes/parents represent the full tree for visualization.
        """
        sx, sy = start
        gx, gy = goal

        if not self.is_collision_free(sx, sy, dynamic_obs):
            return None, [], {}
        if not self.is_collision_free(gx, gy, dynamic_obs):
            return None, [], {}

        # nodes[i] = (x, y), costs[i] = cost-to-come, parents[i] = parent index
        nodes = [(sx, sy)]
        costs = [0.0]
        parents = {0: -1}

        goal_idx = None
        best_goal_cost = float('inf')

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

            # Steer toward sample
            step = min(self.rrt_step, dist)
            new_x = nx + (dx / dist) * step
            new_y = ny + (dy / dist) * step

            if not self.is_edge_free(nx, ny, new_x, new_y, dynamic_obs):
                continue

            # Find near nodes for RRT* rewiring
            n_nodes = len(nodes) + 1
            r = min(self.rewire_radius * math.sqrt(math.log(n_nodes) / n_nodes),
                    self.rewire_radius)
            r = max(r, self.rrt_step)

            near_indices = []
            for i, (nd_x, nd_y) in enumerate(nodes):
                if self._dist(nd_x, nd_y, new_x, new_y) <= r:
                    near_indices.append(i)

            # Choose best parent from near nodes
            best_parent = nearest_idx
            best_cost = costs[nearest_idx] + self._dist(nx, ny, new_x, new_y)

            for ni in near_indices:
                nd_x, nd_y = nodes[ni]
                new_cost = costs[ni] + self._dist(nd_x, nd_y, new_x, new_y)
                if new_cost < best_cost:
                    if self.is_edge_free(nd_x, nd_y, new_x, new_y, dynamic_obs):
                        best_parent = ni
                        best_cost = new_cost

            # Add new node
            new_idx = len(nodes)
            nodes.append((new_x, new_y))
            costs.append(best_cost)
            parents[new_idx] = best_parent

            # Rewire near nodes through new node if cheaper
            for ni in near_indices:
                nd_x, nd_y = nodes[ni]
                rewire_cost = best_cost + self._dist(new_x, new_y, nd_x, nd_y)
                if rewire_cost < costs[ni]:
                    if self.is_edge_free(new_x, new_y, nd_x, nd_y, dynamic_obs):
                        parents[ni] = new_idx
                        costs[ni] = rewire_cost
                        # Propagate cost to children
                        self._propagate_cost(ni, nodes, costs, parents)

            # Check if we can connect to goal
            d_to_goal = self._dist(new_x, new_y, gx, gy)
            if d_to_goal < self.rrt_step:
                if self.is_edge_free(new_x, new_y, gx, gy, dynamic_obs):
                    new_goal_cost = best_cost + d_to_goal
                    if new_goal_cost < best_goal_cost:
                        if goal_idx is None:
                            goal_idx = len(nodes)
                            nodes.append((gx, gy))
                            costs.append(new_goal_cost)
                        else:
                            costs[goal_idx] = new_goal_cost
                        parents[goal_idx] = new_idx
                        best_goal_cost = new_goal_cost

        if goal_idx is None:
            return None, nodes, parents

        # Reconstruct path
        path = []
        idx = goal_idx
        while idx != -1:
            path.append(nodes[idx])
            idx = parents[idx]
        path.reverse()

        return path, nodes, parents

    def _propagate_cost(self, node_idx, nodes, costs, parents):
        """Recursively update costs for children after rewiring."""
        for i, p in parents.items():
            if p == node_idx:
                nd_x, nd_y = nodes[i]
                px, py = nodes[node_idx]
                costs[i] = costs[node_idx] + self._dist(px, py, nd_x, nd_y)
                self._propagate_cost(i, nodes, costs, parents)

    def publish_rrt_tree(self, nodes, parents, final_path):
        """Publish RRT* tree edges and final path as Markers."""
        now = self.get_clock().now().to_msg()

        # Tree edges (blue, thin)
        tree_marker = Marker()
        tree_marker.header.frame_id = 'map'
        tree_marker.header.stamp = now
        tree_marker.ns = 'rrt_tree'
        tree_marker.id = 0
        tree_marker.type = Marker.LINE_LIST
        tree_marker.action = Marker.ADD
        tree_marker.scale.x = 0.02  # line width
        tree_marker.color = ColorRGBA(r=0.3, g=0.5, b=1.0, a=0.4)
        tree_marker.pose.orientation.w = 1.0

        for idx, parent_idx in parents.items():
            if parent_idx == -1 or idx >= len(nodes) or parent_idx >= len(nodes):
                continue
            cx, cy = nodes[idx]
            px, py = nodes[parent_idx]
            tree_marker.points.append(Point(x=px, y=py, z=0.05))
            tree_marker.points.append(Point(x=cx, y=cy, z=0.05))

        self.rrt_tree_pub.publish(tree_marker)

        # Final path (yellow, thick)
        if final_path:
            path_marker = Marker()
            path_marker.header.frame_id = 'map'
            path_marker.header.stamp = now
            path_marker.ns = 'rrt_path'
            path_marker.id = 0
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.scale.x = 0.06
            path_marker.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.9)
            path_marker.pose.orientation.w = 1.0

            for x, y in final_path:
                path_marker.points.append(Point(x=x, y=y, z=0.05))

            self.rrt_path_pub.publish(path_marker)

    def clear_rrt_markers(self):
        """Clear RRT markers when path is clear."""
        now = self.get_clock().now().to_msg()
        for pub, ns in [(self.rrt_tree_pub, 'rrt_tree'),
                        (self.rrt_path_pub, 'rrt_path')]:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = now
            marker.ns = ns
            marker.id = 0
            marker.action = Marker.DELETE
            pub.publish(marker)

    def find_closest_idx(self, rx, ry, poses):
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
            self.local_path_pub.publish(self.global_path)
            return

        rx, ry, ryaw = pose
        poses = self.global_path.poses

        # Convert scan to map-frame obstacles
        scan_points = self.scan_to_obstacles(rx, ry, ryaw)
        dynamic_obs = self.find_dynamic_obstacles(scan_points)

        # Find closest point on global path
        closest_idx = self.find_closest_idx(rx, ry, poses)

        # Check ahead on global path
        ahead_poses = poses[closest_idx:]
        blocked, block_rel_idx = self.is_path_blocked(rx, ry, ahead_poses, dynamic_obs)

        if not blocked:
            self.local_path_pub.publish(self.global_path)
            self.clear_rrt_markers()
            self.get_logger().info(
                f'Path clear. dynamic_obs={len(dynamic_obs)}, scan_pts={len(scan_points)}',
                throttle_duration_sec=2.0
            )
            return

        block_idx = closest_idx + block_rel_idx
        bx_pos = poses[block_idx].pose.position.x
        by_pos = poses[block_idx].pose.position.y
        self.get_logger().info(
            f'Path BLOCKED at idx {block_idx} ({bx_pos:.1f},{by_pos:.1f}), '
            f'dynamic_obs={len(dynamic_obs)}, running RRT*...',
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

        # Run RRT*
        rrt_path, tree_nodes, tree_parents = self.rrt_star(
            (rx, ry), (gx, gy), dynamic_obs
        )

        # Publish tree visualization
        self.publish_rrt_tree(tree_nodes, tree_parents, rrt_path)

        if rrt_path is None:
            self.get_logger().warn(
                f'RRT* failed ({len(tree_nodes)} nodes explored), '
                f'publishing truncated path to STOP before obstacle',
                throttle_duration_sec=1.0
            )
            # Publish truncated path that stops before the blocked point
            # so pure pursuit slows down and stops instead of driving into obstacle
            stop_msg = Path()
            stop_msg.header.frame_id = 'map'
            stop_msg.header.stamp = self.get_clock().now().to_msg()
            # Include path up to a few waypoints before the block
            stop_idx = max(closest_idx, block_idx - 3)
            for p in poses[closest_idx:stop_idx + 1]:
                stop_msg.poses.append(p)
            # If no poses to publish, just publish current position as a "stay here" path
            if len(stop_msg.poses) == 0:
                p = PoseStamped()
                p.header.frame_id = 'map'
                p.pose.position.x = rx
                p.pose.position.y = ry
                p.pose.orientation.w = 1.0
                stop_msg.poses.append(p)
            self.local_path_pub.publish(stop_msg)
            return

        # Build local path: RRT* detour + remaining global path
        local_msg = Path()
        local_msg.header.frame_id = 'map'
        local_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in rrt_path:
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.w = 1.0
            local_msg.poses.append(p)

        for p in poses[rejoin_idx + 1:]:
            local_msg.poses.append(p)

        self.local_path_pub.publish(local_msg)
        self.get_logger().info(
            f'RRT* detour: {len(rrt_path)} path pts, '
            f'{len(tree_nodes)} tree nodes explored',
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
