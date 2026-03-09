import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
import heapq
import yaml
import os
from PIL import Image
from ament_index_python.packages import get_package_share_directory
from scipy.ndimage import binary_dilation


class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner')

        # Parameters
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('inflation_radius', 0.5)  # meters
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('waypoint_spacing', 0.2)  # meters between waypoints
        self.declare_parameter('weight_smooth', 0.3)
        self.declare_parameter('weight_data', 0.1)
        self.declare_parameter('smooth_iterations', 400)
        self.declare_parameter('waypoints', [
            -10.0, -5.0,    # left side
            -10.0, 1.0,     # top-left
            10.0, 1.0,      # top-right
            10.0, -12.0,    # bottom-right
            -10.0, -12.0,   # bottom-left
            -10.0, -5.0,    # back to start (loop)
        ])

        # Load map
        map_yaml = self.get_parameter('map_yaml').value
        if not map_yaml:
            pkg_dir = get_package_share_directory('ee_cs_265a')
            map_yaml = os.path.join(pkg_dir, 'maps', 'track_map.yaml')

        self.load_map(map_yaml)

        # Publishers
        self.path_pub = self.create_publisher(Path, '/global_path', 10)
        self.inflated_map_pub = self.create_publisher(OccupancyGrid, '/inflated_map', 10)

        # Compute path once
        self.compute_global_path()

        # Publish path periodically
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.publish_path)
        self.get_logger().info('Global planner ready, publishing path')

    def load_map(self, yaml_path):
        with open(yaml_path, 'r') as f:
            map_meta = yaml.safe_load(f)

        self.resolution = map_meta['resolution']
        self.origin_x = map_meta['origin'][0]
        self.origin_y = map_meta['origin'][1]
        occupied_thresh = map_meta['occupied_thresh']
        free_thresh = map_meta['free_thresh']

        map_dir = os.path.dirname(yaml_path)
        img_path = os.path.join(map_dir, map_meta['image'])
        img = Image.open(img_path)
        raw = np.array(img, dtype=np.float64) / 255.0

        # Build occupancy: 0=free, 1=occupied, -1=unknown
        self.height, self.width = raw.shape
        self.occupancy = np.full((self.height, self.width), -1, dtype=np.int8)
        self.occupancy[raw >= (1.0 - free_thresh)] = 0       # free
        self.occupancy[raw <= (1.0 - occupied_thresh)] = 100  # occupied

        # Flip vertically (PGM origin is top-left, map origin is bottom-left)
        self.occupancy = np.flipud(self.occupancy)

        n_free = np.sum(self.occupancy == 0)
        n_occ = np.sum(self.occupancy == 100)
        n_unk = np.sum(self.occupancy == -1)
        self.get_logger().info(
            f'Map loaded: {self.width}x{self.height}, '
            f'resolution={self.resolution}m, '
            f'origin=({self.origin_x}, {self.origin_y}), '
            f'free_thresh={free_thresh}, '
            f'free={n_free}, occ={n_occ}, unknown={n_unk}'
        )

        # Inflate obstacles
        self.inflate_obstacles()

    def inflate_obstacles(self):
        inflation_radius = self.get_parameter('inflation_radius').value
        inflation_cells = int(np.ceil(inflation_radius / self.resolution))

        occupied_mask = (self.occupancy == 100)
        unknown_mask = (self.occupancy == -1)
        obstacle_mask = occupied_mask | unknown_mask

        # Create circular structuring element
        y, x = np.ogrid[-inflation_cells:inflation_cells + 1,
                         -inflation_cells:inflation_cells + 1]
        struct = (x * x + y * y) <= (inflation_cells * inflation_cells)

        inflated = binary_dilation(obstacle_mask, structure=struct)

        self.inflated_occ = self.occupancy.copy()
        self.inflated_occ[inflated] = 100
        # Keep original free cells that weren't inflated
        self.free_mask = (self.inflated_occ == 0)

        n_free = np.sum(self.free_mask)
        self.get_logger().info(
            f'Inflated obstacles by {inflation_radius}m '
            f'({inflation_cells} cells). Free cells: {n_free}'
        )

    def world_to_grid(self, wx, wy):
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = gx * self.resolution + self.origin_x + self.resolution / 2
        wy = gy * self.resolution + self.origin_y + self.resolution / 2
        return wx, wy

    def is_valid(self, gx, gy):
        return (0 <= gx < self.width and 0 <= gy < self.height
                and self.inflated_occ[gy, gx] == 0)

    def astar(self, start_grid, goal_grid):
        sx, sy = start_grid
        gx, gy = goal_grid

        if not self.is_valid(sx, sy):
            self.get_logger().error(
                f'Start ({sx},{sy}) is in obstacle! Searching nearby free cell...'
            )
            sx, sy = self.find_nearest_free(sx, sy)
            if sx is None:
                return None

        if not self.is_valid(gx, gy):
            self.get_logger().error(
                f'Goal ({gx},{gy}) is in obstacle! Searching nearby free cell...'
            )
            gx, gy = self.find_nearest_free(gx, gy)
            if gx is None:
                return None

        # A* with 8-connected grid
        open_set = []
        heapq.heappush(open_set, (0, sx, sy))
        came_from = {}
        g_score = {(sx, sy): 0}
        closed = set()

        # 8 directions
        dirs = [(-1, 0), (1, 0), (0, -1), (0, 1),
                (-1, -1), (-1, 1), (1, -1), (1, 1)]
        costs = [1.0, 1.0, 1.0, 1.0,
                 1.414, 1.414, 1.414, 1.414]

        while open_set:
            _, cx, cy = heapq.heappop(open_set)

            if (cx, cy) in closed:
                continue
            closed.add((cx, cy))

            if cx == gx and cy == gy:
                # Reconstruct path
                path = [(gx, gy)]
                while (path[-1][0], path[-1][1]) in came_from:
                    path.append(came_from[(path[-1][0], path[-1][1])])
                path.reverse()
                return path

            for (dx, dy), cost in zip(dirs, costs):
                nx, ny = cx + dx, cy + dy
                if not self.is_valid(nx, ny):
                    continue
                if (nx, ny) in closed:
                    continue

                tentative_g = g_score[(cx, cy)] + cost
                if tentative_g < g_score.get((nx, ny), float('inf')):
                    came_from[(nx, ny)] = (cx, cy)
                    g_score[(nx, ny)] = tentative_g
                    h = np.sqrt((nx - gx) ** 2 + (ny - gy) ** 2)
                    f = tentative_g + h
                    heapq.heappush(open_set, (f, nx, ny))

        self.get_logger().error('A* failed to find a path!')
        return None

    def find_nearest_free(self, gx, gy, max_radius=50):
        for r in range(1, max_radius):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if abs(dx) == r or abs(dy) == r:
                        nx, ny = gx + dx, gy + dy
                        if self.is_valid(nx, ny):
                            self.get_logger().info(
                                f'Found free cell at ({nx},{ny}), '
                                f'offset ({dx},{dy})'
                            )
                            return nx, ny
        self.get_logger().error('No free cell found nearby!')
        return None, None

    def smooth_path(self, path):
        """Gradient-descent path smoother. Keeps endpoints fixed.
        Updates both x and y atomically to avoid invalid intermediate states."""
        weight_smooth = self.get_parameter('weight_smooth').value
        weight_data = self.get_parameter('weight_data').value
        max_iter = int(self.get_parameter('smooth_iterations').value)
        tolerance = 1e-4

        smoothed = np.array(path, dtype=np.float64)
        original = smoothed.copy()

        for iteration in range(max_iter):
            change = 0.0
            for i in range(1, len(smoothed) - 1):
                old_x, old_y = smoothed[i][0], smoothed[i][1]

                # Compute update for BOTH axes first
                new_x = old_x + (
                    weight_data * (original[i][0] - old_x)
                    + weight_smooth * (
                        smoothed[i - 1][0] + smoothed[i + 1][0] - 2.0 * old_x
                    )
                )
                new_y = old_y + (
                    weight_data * (original[i][1] - old_y)
                    + weight_smooth * (
                        smoothed[i - 1][1] + smoothed[i + 1][1] - 2.0 * old_y
                    )
                )

                # Validate the COMBINED new position
                check_gx = int(round(new_x))
                check_gy = int(round(new_y))
                if self.is_valid(check_gx, check_gy):
                    smoothed[i][0] = new_x
                    smoothed[i][1] = new_y
                    change += abs(new_x - old_x) + abs(new_y - old_y)
                # else: keep old position entirely

            if change < tolerance:
                self.get_logger().info(
                    f'Path smoothing converged after {iteration + 1} iterations'
                )
                break

        # Convert back to integer grid coords
        result = [(int(round(p[0])), int(round(p[1]))) for p in smoothed]
        self.get_logger().info(f'Smoothed path: {len(result)} points')
        return result

    def compute_global_path(self):
        # Parse waypoints from flat list [x1,y1, x2,y2, ...]
        wp_flat = self.get_parameter('waypoints').value
        waypoints = [(wp_flat[i], wp_flat[i + 1])
                     for i in range(0, len(wp_flat), 2)]

        self.get_logger().info(
            f'Planning loop through {len(waypoints)} waypoints'
        )

        # Plan A* between consecutive waypoints
        full_grid_path = []
        for i in range(len(waypoints) - 1):
            sx, sy = waypoints[i]
            gx, gy = waypoints[i + 1]
            start_grid = self.world_to_grid(sx, sy)
            goal_grid = self.world_to_grid(gx, gy)

            self.get_logger().info(
                f'  Segment {i}: ({sx},{sy}) -> ({gx},{gy})'
            )

            segment = self.astar(start_grid, goal_grid)
            if segment is None:
                self.get_logger().error(
                    f'  Failed on segment {i}!'
                )
                self.global_path = None
                return

            # Append segment (skip first point to avoid duplicates)
            if full_grid_path:
                full_grid_path.extend(segment[1:])
            else:
                full_grid_path.extend(segment)

        # Smooth the path
        full_grid_path = self.smooth_path(full_grid_path)

        # Downsample path based on waypoint spacing
        spacing = self.get_parameter('waypoint_spacing').value
        spacing_cells = max(1, int(spacing / self.resolution))

        sampled = [full_grid_path[i]
                   for i in range(0, len(full_grid_path), spacing_cells)]
        if sampled[-1] != full_grid_path[-1]:
            sampled.append(full_grid_path[-1])

        # Build Path message
        self.global_path = Path()
        self.global_path.header.frame_id = 'map'

        for gx, gy in sampled:
            wx, wy = self.grid_to_world(gx, gy)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            self.global_path.poses.append(pose)

        self.get_logger().info(
            f'Full loop path: {len(full_grid_path)} cells -> '
            f'{len(self.global_path.poses)} waypoints'
        )

    def publish_path(self):
        if self.global_path is None:
            return

        now = self.get_clock().now().to_msg()
        self.global_path.header.stamp = now
        for pose in self.global_path.poses:
            pose.header.stamp = now
        self.path_pub.publish(self.global_path)

        # Also publish inflated map for visualization
        self.publish_inflated_map()

    def publish_inflated_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = float(self.origin_x)
        msg.info.origin.position.y = float(self.origin_y)

        # Convert to OccupancyGrid format: 0=free, 100=occupied, -1=unknown
        data = self.inflated_occ.flatten().tolist()
        msg.data = data
        self.inflated_map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
