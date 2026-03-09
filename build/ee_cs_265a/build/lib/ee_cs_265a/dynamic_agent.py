import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class DynamicAgent(Node):
    """Scripted obstacle car that follows a fixed waypoint loop.

    Drives the car along a sequence of waypoints using simple
    proportional steering. Used to create moving obstacles for
    the red car's local planner to avoid.
    """

    def __init__(self):
        super().__init__('dynamic_agent')

        self.declare_parameter('cmd_vel_topic', '/blue/cmd_vel')
        self.declare_parameter('odom_topic', '/blue/odometry')
        self.declare_parameter('speed', 0.6)
        self.declare_parameter('waypoint_tolerance', 1.5)
        self.declare_parameter('kp_angular', 2.0)
        self.declare_parameter('max_omega', 3.0)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('loop', True)
        # Flat list: x1,y1, x2,y2, ...
        self.declare_parameter('waypoints', [0.0])

        cmd_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.speed = self.get_parameter('speed').value
        self.wp_tol = self.get_parameter('waypoint_tolerance').value
        self.kp = self.get_parameter('kp_angular').value
        self.max_omega = self.get_parameter('max_omega').value
        self.do_loop = self.get_parameter('loop').value
        rate = self.get_parameter('control_rate').value

        # Parse waypoints
        wp_flat = self.get_parameter('waypoints').value
        self.waypoints = [(wp_flat[i], wp_flat[i + 1])
                          for i in range(0, len(wp_flat) - 1, 2)]
        self.current_wp = 0
        self.finished = False

        # Odom state
        self.x = None
        self.y = None
        self.yaw = None

        # The odom frame starts at the car's spawn position.
        # We need to know the spawn position to convert odom to world coords.
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 10.0)
        self.declare_parameter('spawn_yaw', 0.0)
        self.spawn_x = self.get_parameter('spawn_x').value
        self.spawn_y = self.get_parameter('spawn_y').value
        self.spawn_yaw = self.get_parameter('spawn_yaw').value

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(
            f'Dynamic agent: {len(self.waypoints)} waypoints, '
            f'speed={self.speed}, topic={cmd_topic}'
        )

    def odom_cb(self, msg):
        # Odom position is relative to spawn
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        # Convert to world frame
        cos_s = math.cos(self.spawn_yaw)
        sin_s = math.sin(self.spawn_yaw)
        self.x = self.spawn_x + cos_s * ox - sin_s * oy
        self.y = self.spawn_y + sin_s * ox + cos_s * oy
        self.yaw = self.spawn_yaw + odom_yaw

    def control_loop(self):
        if self.x is None or self.finished:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return

        if self.current_wp >= len(self.waypoints):
            if self.do_loop:
                self.current_wp = 0
            else:
                self.finished = True
                self.cmd_pub.publish(Twist())
                self.get_logger().info('Agent finished all waypoints')
                return

        gx, gy = self.waypoints[self.current_wp]
        dx = gx - self.x
        dy = gy - self.y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < self.wp_tol:
            self.current_wp += 1
            return

        # Proportional steering toward waypoint
        target_yaw = math.atan2(dy, dx)
        err = target_yaw - self.yaw
        err = math.atan2(math.sin(err), math.cos(err))  # normalize

        omega = self.kp * err
        omega = max(-self.max_omega, min(self.max_omega, omega))

        # Slow down when turning hard
        speed = self.speed
        if abs(err) > math.pi / 3:
            speed *= 0.5

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicAgent()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
