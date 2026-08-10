import math

import message_filters
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from tf2_ros import TransformException

# Best-effort, shallow-queue QoS: a dropped or stale keyframe should never
# block or back up the low-latency control topics sharing the DDS link.
_KEYFRAME_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=2,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class KeyframePublisherNode(Node):
    """Selects and republishes posed RGB-D keyframes for off-board 3DGS mapping.

    Pairs synchronized color/depth frames from oakd_publisher with the
    map -> camera_frame pose (via tf2, populated by the onboard EKF + off-board
    SLAM Toolbox) and only republishes when the rover has moved enough since the
    last keyframe. Downstream (off-board) consumers pair color/depth/pose by
    matching header stamps.
    """

    def __init__(self):
        super().__init__('keyframe_publisher_node')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', 'oakd_camera_optical_frame')
        self.declare_parameter('min_translation_m', 0.15)
        self.declare_parameter('min_rotation_rad', 0.20)

        self.map_frame = self.get_parameter('map_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.min_translation_m = float(self.get_parameter('min_translation_m').value)
        self.min_rotation_rad = float(self.get_parameter('min_rotation_rad').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.color_pub = self.create_publisher(Image, '/keyframe/color', _KEYFRAME_QOS)
        self.depth_pub = self.create_publisher(Image, '/keyframe/depth', _KEYFRAME_QOS)
        self.pose_pub = self.create_publisher(PoseStamped, '/keyframe/pose', _KEYFRAME_QOS)

        self._last_keyframe_xyz = None
        self._last_keyframe_yaw = None

        color_sub = message_filters.Subscriber(self, Image, '/oakd/color/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/oakd/depth/image_raw')
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub], queue_size=5, slop=0.05,
        )
        self._sync.registerCallback(self._on_frame_pair)

    def _on_frame_pair(self, color_msg: Image, depth_msg: Image):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.camera_frame, color_msg.header.stamp,
            )
        except TransformException as exc:
            self.get_logger().debug(f'tf lookup failed, skipping keyframe: {exc}')
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        if not self._is_new_keyframe((t.x, t.y, t.z), yaw):
            return

        self._last_keyframe_xyz = (t.x, t.y, t.z)
        self._last_keyframe_yaw = yaw

        pose_msg = PoseStamped()
        pose_msg.header.stamp = color_msg.header.stamp
        pose_msg.header.frame_id = self.map_frame
        pose_msg.pose.position.x = t.x
        pose_msg.pose.position.y = t.y
        pose_msg.pose.position.z = t.z
        pose_msg.pose.orientation = q

        self.color_pub.publish(color_msg)
        self.depth_pub.publish(depth_msg)
        self.pose_pub.publish(pose_msg)

    def _is_new_keyframe(self, xyz, yaw) -> bool:
        if self._last_keyframe_xyz is None:
            return True
        dx = xyz[0] - self._last_keyframe_xyz[0]
        dy = xyz[1] - self._last_keyframe_xyz[1]
        dz = xyz[2] - self._last_keyframe_xyz[2]
        translation = math.sqrt(dx * dx + dy * dy + dz * dz)
        rotation = abs(math.atan2(math.sin(yaw - self._last_keyframe_yaw),
                                   math.cos(yaw - self._last_keyframe_yaw)))
        return translation >= self.min_translation_m or rotation >= self.min_rotation_rad


def main(args=None):
    rclpy.init(args=args)
    node = KeyframePublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
