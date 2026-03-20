import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import tf2_ros


class GtLocalizer(Node):
    """Fake perfect localization using Gazebo's ground-truth model pose.

    Subscribes to the PosePublisher bridge (world-frame pose of the car)
    and wheel odometry, then publishes a corrected map->odom TF so that
    map->base_link is always accurate.
    """

    def __init__(self):
        super().__init__('gt_localizer')

        self.declare_parameter('gt_topic', '/red/gt_pose')
        self.declare_parameter('odom_topic', '/red/odometry')
        self.declare_parameter('model_name', 'car1_red')
        self.declare_parameter('rate', 50.0)

        gt_topic = self.get_parameter('gt_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.model_name = self.get_parameter('model_name').value
        rate = self.get_parameter('rate').value

        # Ground truth pose from Gazebo PosePublisher (as raw TFMessage)
        self.gt_tf = None
        self.create_subscription(TFMessage, gt_topic, self.gt_cb, 10)

        # Drifted wheel odometry
        self.odom = None
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)

        # TF broadcaster for map -> odom
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_timer(1.0 / rate, self.publish_correction)

        self.logged_frames = False
        self.get_logger().info(
            f'GT localizer ready: listening for model "{self.model_name}" '
            f'on {gt_topic}'
        )

    def gt_cb(self, msg):
        for tf in msg.transforms:
            # Match the model root transform (exact or contains)
            if tf.child_frame_id == self.model_name or \
               self.model_name in tf.child_frame_id:
                self.gt_tf = tf
                if not self.logged_frames:
                    self.get_logger().info(
                        f'GT pose found: {tf.header.frame_id} -> '
                        f'{tf.child_frame_id} '
                        f'({tf.transform.translation.x:.2f}, '
                        f'{tf.transform.translation.y:.2f})'
                    )
                    self.logged_frames = True
                break
        if self.gt_tf is None and not self.logged_frames:
            frames = [t.child_frame_id for t in msg.transforms[:5]]
            self.get_logger().warn(
                f'No match for "{self.model_name}" in: {frames}',
                throttle_duration_sec=5.0
            )

    def odom_cb(self, msg):
        self.odom = msg

    def publish_correction(self):
        if self.gt_tf is None or self.odom is None:
            return

        # Ground truth: world -> car model origin (= base_link in world frame)
        gt_x = self.gt_tf.transform.translation.x
        gt_y = self.gt_tf.transform.translation.y
        gt_qz = self.gt_tf.transform.rotation.z
        gt_qw = self.gt_tf.transform.rotation.w
        gt_yaw = self._yaw(gt_qw, gt_qz)

        # Drifted odom: odom -> base_link (from wheel encoder integration)
        odom_x = self.odom.pose.pose.position.x
        odom_y = self.odom.pose.pose.position.y
        odom_qz = self.odom.pose.pose.orientation.z
        odom_qw = self.odom.pose.pose.orientation.w
        odom_yaw = self._yaw(odom_qw, odom_qz)

        # map->odom = map->base_link_true * inverse(odom->base_link)
        dyaw = gt_yaw - odom_yaw
        cos_d = math.cos(dyaw)
        sin_d = math.sin(dyaw)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = gt_x - (cos_d * odom_x - sin_d * odom_y)
        t.transform.translation.y = gt_y - (sin_d * odom_x + cos_d * odom_y)
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(dyaw / 2.0)
        t.transform.rotation.w = math.cos(dyaw / 2.0)

        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _yaw(qw, qz):
        return 2.0 * math.atan2(qz, qw)


def main(args=None):
    rclpy.init(args=args)
    node = GtLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
