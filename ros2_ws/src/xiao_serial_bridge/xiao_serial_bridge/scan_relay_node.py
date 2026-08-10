import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanRelayNode(Node):
    """Republish /scan with a fixed beam count and optional FOV crop.

    Handles the LD06's 0→2π angle convention: beams are normalized to
    [-π, π] before FOV filtering so 0° always means "forward".
    """

    def __init__(self):
        super().__init__('scan_relay_node')
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_fixed')
        self.declare_parameter('target_beams', 300)
        self.declare_parameter('fov_deg', 250.0)      # degrees to keep centred at 0°
        self.declare_parameter('fov_center_deg', 0.0)  # 0 = forward

        self._n = self.get_parameter('target_beams').get_parameter_value().integer_value
        fov_deg = self.get_parameter('fov_deg').get_parameter_value().double_value
        center_deg = self.get_parameter('fov_center_deg').get_parameter_value().double_value
        self._half_fov = math.radians(fov_deg / 2.0)
        self._center = math.radians(center_deg)

        in_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self._pub = self.create_publisher(LaserScan, out_topic, 10)
        self._sub = self.create_subscription(LaserScan, in_topic, self._cb, 10)
        self.get_logger().info(
            f'Relaying {in_topic} → {out_topic} | {self._n} beams | FOV {fov_deg:.0f}° centred at {center_deg:.0f}°'
        )

    def _cb(self, msg: LaserScan):
        src_n = len(msg.ranges)
        if src_n == 0:
            return

        half_fov = self._half_fov
        center = self._center

        # Collect beams within the FOV, normalising raw angles to [-π, π]
        kept = []
        for i in range(src_n):
            raw = msg.angle_min + i * msg.angle_increment
            norm = (raw - center + math.pi) % (2.0 * math.pi) - math.pi
            if abs(norm) <= half_fov:
                kept.append((norm, msg.ranges[i]))

        if len(kept) < 2:
            return

        # Sort by normalised angle so beams are in ascending order
        kept.sort(key=lambda x: x[0])
        kept_angles = [p[0] for p in kept]
        kept_ranges = [p[1] for p in kept]

        # Nearest-neighbour resample to target_beams
        n = self._n
        src_k = len(kept_angles)
        out_angle_min = kept_angles[0]
        out_angle_max = kept_angles[-1]
        out_increment = (out_angle_max - out_angle_min) / (n - 1)

        out_ranges = []
        for i in range(n):
            src_idx = int(round(i * (src_k - 1) / (n - 1)))
            out_ranges.append(kept_ranges[src_idx])

        out = LaserScan()
        out.header = msg.header
        out.angle_min = out_angle_min
        out.angle_max = out_angle_max
        out.angle_increment = out_increment
        out.time_increment = msg.time_increment * src_k / n
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = out_ranges
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
