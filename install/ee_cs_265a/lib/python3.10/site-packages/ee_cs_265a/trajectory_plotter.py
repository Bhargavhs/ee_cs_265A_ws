import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')

        self.declare_parameter('record_interval', 0.2)
        self.declare_parameter('save_path', '/tmp/trajectory_plot.png')

        self.save_path = self.get_parameter('save_path').value
        interval = self.get_parameter('record_interval').value

        # TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Recorded data - red car (map-frame via TF)
        self.robot_xs = []
        self.robot_ys = []
        self.path_xs = []
        self.path_ys = []
        self.path_received = False

        # Blue car trajectory (odom + spawn offset)
        self.blue_xs = []
        self.blue_ys = []
        self.blue_last_odom = None
        self.blue_spawn_x = 0.0
        self.blue_spawn_y = 10.0
        self.blue_spawn_yaw = 0.0

        # Green car trajectory (odom + spawn offset)
        self.green_xs = []
        self.green_ys = []
        self.green_last_odom = None
        self.green_spawn_x = 0.0
        self.green_spawn_y = 10.8
        self.green_spawn_yaw = 0.0

        # Subscriptions
        self.create_subscription(Odometry, '/red/odometry', self.odom_cb, 10)
        self.create_subscription(Path, '/global_path', self.path_cb, 10)
        self.create_subscription(Odometry, '/blue/odometry', self.blue_odom_cb, 10)
        self.create_subscription(Odometry, '/green/odometry', self.green_odom_cb, 10)

        self.last_odom = None
        self.timer = self.create_timer(interval, self.record)
        self.save_timer = self.create_timer(10.0, self.save_plot)

        self.get_logger().info('Trajectory plotter ready, will save to ' + self.save_path)

    def odom_cb(self, msg):
        self.last_odom = msg

    def blue_odom_cb(self, msg):
        self.blue_last_odom = msg

    def green_odom_cb(self, msg):
        self.green_last_odom = msg

    def path_cb(self, msg):
        if not self.path_received and len(msg.poses) > 0:
            self.path_xs = [p.pose.position.x for p in msg.poses]
            self.path_ys = [p.pose.position.y for p in msg.poses]
            self.path_received = True
            self.get_logger().info(f'Global path recorded: {len(msg.poses)} points')

    def _odom_to_world(self, odom_msg, spawn_x, spawn_y, spawn_yaw):
        """Convert odom-frame pose to world frame using spawn offset."""
        ox = odom_msg.pose.pose.position.x
        oy = odom_msg.pose.pose.position.y
        cos_s = math.cos(spawn_yaw)
        sin_s = math.sin(spawn_yaw)
        wx = spawn_x + cos_s * ox - sin_s * oy
        wy = spawn_y + sin_s * ox + cos_s * oy
        return wx, wy

    def record(self):
        # Record red car (via TF for AMCL-corrected pose)
        if self.last_odom is not None:
            pose_stamped = PoseStamped()
            pose_stamped.header = self.last_odom.header
            pose_stamped.pose = self.last_odom.pose.pose

            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', pose_stamped.header.frame_id,
                    rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
                )
                transformed = do_transform_pose_stamped(pose_stamped, transform)
                self.robot_xs.append(transformed.pose.position.x)
                self.robot_ys.append(transformed.pose.position.y)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                pass

        # Record blue car (odom + spawn offset)
        if self.blue_last_odom is not None:
            bx, by = self._odom_to_world(
                self.blue_last_odom,
                self.blue_spawn_x, self.blue_spawn_y, self.blue_spawn_yaw
            )
            self.blue_xs.append(bx)
            self.blue_ys.append(by)

        # Record green car (odom + spawn offset)
        if self.green_last_odom is not None:
            gx, gy = self._odom_to_world(
                self.green_last_odom,
                self.green_spawn_x, self.green_spawn_y, self.green_spawn_yaw
            )
            self.green_xs.append(gx)
            self.green_ys.append(gy)

    def save_plot(self):
        if len(self.robot_xs) < 2:
            return

        fig, ax = plt.subplots(1, 1, figsize=(10, 12))

        # Outer walls
        outer = plt.Rectangle((-10, -12), 20, 24, fill=False, edgecolor='black', linewidth=2, label='Outer wall')
        ax.add_patch(outer)
        # Upper island
        upper = plt.Rectangle((-5, 4), 10, 4, fill=True, facecolor='lightyellow', edgecolor='orange', linewidth=1.5, label='Upper island')
        ax.add_patch(upper)
        # Lower island
        lower = plt.Rectangle((-5, -8), 10, 4, fill=True, facecolor='lightyellow', edgecolor='orange', linewidth=1.5, label='Lower island')
        ax.add_patch(lower)
        # Center block
        center = plt.Rectangle((-1.5, -1.5), 3, 3, fill=True, facecolor='mistyrose', edgecolor='darkred', linewidth=1.5, label='Center block')
        ax.add_patch(center)

        if self.path_received:
            ax.plot(self.path_xs, self.path_ys, 'g--', linewidth=1.5, alpha=0.7, label='Global path')

        # Red car trajectory
        ax.plot(self.robot_xs, self.robot_ys, 'r-', linewidth=1.5, label='Red (ego)')
        ax.plot(self.robot_xs[0], self.robot_ys[0], 'ro', markersize=8)
        ax.plot(self.robot_xs[-1], self.robot_ys[-1], 'r*', markersize=15)

        # Blue car trajectory
        if len(self.blue_xs) > 1:
            ax.plot(self.blue_xs, self.blue_ys, 'b-', linewidth=1.2, alpha=0.7, label='Blue (obstacle)')
            ax.plot(self.blue_xs[-1], self.blue_ys[-1], 'b*', markersize=12)

        # Green car trajectory
        if len(self.green_xs) > 1:
            ax.plot(self.green_xs, self.green_ys, 'g-', linewidth=1.2, alpha=0.7, label='Green (obstacle)')
            ax.plot(self.green_xs[-1], self.green_ys[-1], 'g*', markersize=12)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Robot Trajectory vs Global Path')
        ax.legend(loc='upper right')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlim(-12, 12)
        ax.set_ylim(-14, 14)

        fig.savefig(self.save_path, dpi=150, bbox_inches='tight')
        plt.close(fig)
        self.get_logger().info(
            f'Plot saved: {self.save_path} '
            f'(red={len(self.robot_xs)}, blue={len(self.blue_xs)}, green={len(self.green_xs)} pts)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_plot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
