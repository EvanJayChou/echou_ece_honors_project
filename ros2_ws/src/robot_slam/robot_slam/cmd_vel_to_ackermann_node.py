import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelToAckermannNode(Node):
    """Convert Nav2's /cmd_vel Twist into /ackermann_cmd for ackermann_to_vesc.

    Uses the bicycle model: steering = atan(wheelbase * w / v). A Twist with
    v ≈ 0 and w ≠ 0 (rotate in place) has no Ackermann equivalent, so it is sent
    as zero speed with centered steering. The controller is configured never to
    ask for that (RPP with use_rotate_to_heading: false).
    """

    def __init__(self):
        super().__init__('cmd_vel_to_ackermann_node')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/ackermann_cmd')
        self.declare_parameter('wheelbase', 0.34)          # m — matches vesc_odom.yaml
        self.declare_parameter('max_steering_angle', 0.4)  # rad — matches keyboard teleop MAX_STEER
        self.declare_parameter('min_speed_for_steering', 0.02)  # m/s

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.max_steer = float(self.get_parameter('max_steering_angle').value)
        self.min_speed = float(self.get_parameter('min_speed_for_steering').value)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self._pub = self.create_publisher(AckermannDriveStamped, out_topic, 10)
        self._sub = self.create_subscription(Twist, in_topic, self._cb, 10)
        self.get_logger().info(
            f'Converting {in_topic} → {out_topic} | wheelbase {self.wheelbase:.2f} m | '
            f'max steer {self.max_steer:.2f} rad'
        )

    def _cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        steer = 0.0
        if abs(v) >= self.min_speed:
            steer = math.atan(self.wheelbase * w / v)
            steer = max(-self.max_steer, min(self.max_steer, steer))
        elif abs(w) > 1e-3:
            self.get_logger().warning(
                'Rotate-in-place command (v≈0, w≠0) cannot be executed by an Ackermann base',
                throttle_duration_sec=2.0,
            )
            v = 0.0

        out = AckermannDriveStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.drive.speed = float(v)
        out.drive.steering_angle = float(steer)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAckermannNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
