import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        # Parameters
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('min_lookahead', 0.2)
        self.declare_parameter('max_lookahead', 0.9)
        self.declare_parameter('lookahead_speed_gain', 1.0)
        self.declare_parameter('target_speed', 1.0)
        self.declare_parameter('min_speed', 0.3)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('speed_curvature_gain', 0.8)
        self.declare_parameter('max_omega', 20.0)
        self.declare_parameter('goal_tolerance', 1.0)

        self.base_lookahead = self.get_parameter('lookahead_distance').value
        self.min_lookahead = self.get_parameter('min_lookahead').value
        self.max_lookahead = self.get_parameter('max_lookahead').value
        self.lookahead_speed_gain = self.get_parameter('lookahead_speed_gain').value
        self.target_speed = self.get_parameter('target_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.speed_curv_gain = self.get_parameter('speed_curvature_gain').value
        self.max_omega = self.get_parameter('max_omega').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.goal_reached = False

        # TF buffer for odom -> map transform
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State
        self.path = None
        self.odom = None

        # Subscriptions
        self.create_subscription(Path, '/global_path', self.path_cb, 10)
        self.create_subscription(Odometry, '/red/odometry', self.odom_cb, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/red/cmd_vel', 10)

        # Control timer
        rate = self.get_parameter('control_rate').value
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(
            f'Pure Pursuit ready: lookahead={self.base_lookahead}m '
            f'[{self.min_lookahead}-{self.max_lookahead}], '
            f'speed={self.target_speed}m/s'
        )

    def path_cb(self, msg):
        if len(msg.poses) > 0:
            self.path = msg

    def odom_cb(self, msg):
        self.odom = msg

    def get_yaw(self, orientation):
        q = orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def get_current_speed(self):
        vx = self.odom.twist.twist.linear.x
        vy = self.odom.twist.twist.linear.y
        return math.sqrt(vx * vx + vy * vy)

    def get_pose_in_map(self):
        """Transform odom pose to map frame using TF."""
        pose_stamped = PoseStamped()
        pose_stamped.header = self.odom.header
        pose_stamped.pose = self.odom.pose.pose

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', pose_stamped.header.frame_id,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
            )
            transformed = do_transform_pose_stamped(pose_stamped, transform)
            return transformed.pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return None

    def control_loop(self):
        if self.path is None or self.odom is None:
            return

        if self.goal_reached:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return

        # Get car pose in map frame
        map_pose = self.get_pose_in_map()
        if map_pose is None:
            return

        cx = map_pose.position.x
        cy = map_pose.position.y
        yaw = self.get_yaw(map_pose.orientation)
        current_speed = self.get_current_speed()

        poses = self.path.poses
        n = len(poses)

        # Check distance to goal (last point)
        gx = poses[-1].pose.position.x
        gy = poses[-1].pose.position.y
        goal_dist = math.sqrt((gx - cx) ** 2 + (gy - cy) ** 2)
        if goal_dist < self.goal_tolerance:
            self.goal_reached = True
            self.get_logger().info('Goal reached!')
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return

        # Find closest point on path
        min_dist_sq = float('inf')
        closest_idx = 0
        for i in range(n):
            dx = poses[i].pose.position.x - cx
            dy = poses[i].pose.position.y - cy
            d = dx * dx + dy * dy
            if d < min_dist_sq:
                min_dist_sq = d
                closest_idx = i

        # Adaptive lookahead
        lookahead = self.base_lookahead + self.lookahead_speed_gain * current_speed
        lookahead = max(self.min_lookahead, min(self.max_lookahead, lookahead))

        # Walk forward to find lookahead point (clamp at end, don't wrap)
        lookahead_idx = closest_idx
        dist_accum = 0.0
        for _ in range(n):
            nxt = lookahead_idx + 1
            if nxt >= n:
                lookahead_idx = n - 1
                break
            dx = poses[nxt].pose.position.x - poses[lookahead_idx].pose.position.x
            dy = poses[nxt].pose.position.y - poses[lookahead_idx].pose.position.y
            dist_accum += math.sqrt(dx * dx + dy * dy)
            lookahead_idx = nxt
            if dist_accum >= lookahead:
                break

        lx = poses[lookahead_idx].pose.position.x
        ly = poses[lookahead_idx].pose.position.y

        # Pure pursuit geometry
        dx = lx - cx
        dy = ly - cy
        alpha = math.atan2(dy, dx) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        ld = max(math.sqrt(dx * dx + dy * dy), 0.01)
        curvature = 2.0 * math.sin(alpha) / ld

        # Speed: reduce proportionally to curvature
        curv_factor = min(abs(curvature) * 2.0, 1.0)
        speed = self.target_speed * (1.0 - self.speed_curv_gain * curv_factor)
        speed = max(self.min_speed, speed)

        # Slow down near goal
        if goal_dist < 3.0:
            speed = max(self.min_speed, speed * (goal_dist / 3.0))

        # If car is facing away from lookahead, slow down significantly
        if abs(alpha) > math.pi / 2:
            speed = self.min_speed

        # omega from curvature
        omega = speed * curvature
        omega = max(-self.max_omega, min(self.max_omega, omega))

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
