import math
import queue
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError:
    raise ImportError("pyserial is required: pip install pyserial")

# CSV field indices
_SEQ = 0
_TS = 1
_AX, _AY, _AZ = 2, 3, 4
_GX, _GY, _GZ = 5, 6, 7
_X, _Y, _THETA = 8, 9, 10
_VX, _VTHETA = 11, 12
_EXPECTED_FIELDS = 13

# Collect this many gyro samples at startup before declaring bias calibrated.
# 500 ticks × 20 ms = 10 s — robot must be stationary during this window.
_GYRO_CAL_SAMPLES = 500

_POSE_COV = [0.0] * 36
_POSE_COV[0] = 0.05   # x
_POSE_COV[7] = 0.05   # y
_POSE_COV[35] = 0.02  # yaw

_TWIST_COV = [0.0] * 36
_TWIST_COV[0] = 0.5    # vx — high covariance; accel-derived velocity is noisy
_TWIST_COV[35] = 0.01  # vtheta


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)

        self._port = self.get_parameter('serial_port').get_parameter_value().string_value
        self._baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self._odom_frame = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self._base_frame = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self._publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._imu_pub = self.create_publisher(Imu, '/imu', 10)
        if self._publish_tf:
            self._tf_broadcaster = TransformBroadcaster(self)

        self._pkt_queue: queue.Queue = queue.Queue()
        self._serial_thread = threading.Thread(target=self._read_serial, daemon=True)
        self._serial_thread.start()

        self._gyro_bias = (0.0, 0.0, 0.0)
        self._cal_buf: list[tuple[float, float, float]] = []
        self._calibrated = False

        # 50 Hz timer — matches firmware output rate
        self.create_timer(0.02, self._publish_cb)
        self.get_logger().info(f'serial_bridge_node started on {self._port}')
        self.get_logger().info(
            f'Gyro bias calibration: keep robot still for '
            f'{_GYRO_CAL_SAMPLES / 50:.0f} s after first serial packet'
        )

    def _find_xiao_port(self):
        from serial.tools import list_ports
        for p in list_ports.comports():
            if p.vid == 0x2886 and p.pid == 0x8045:
                return p.device
        return None

    def _open_serial(self):
        import time, os
        while rclpy.ok():
            port = self._port
            if not os.path.exists(port):
                detected = self._find_xiao_port()
                if detected:
                    self.get_logger().info(f'{port} not found — auto-detected XIAO on {detected}')
                    port = detected
            try:
                ser = serial.Serial(port, self._baud, timeout=1.0)
                time.sleep(0.1)
                ser.reset_input_buffer()
                self.get_logger().info(f'Opened serial port {port}')
                return ser
            except serial.SerialException as e:
                detected = self._find_xiao_port()
                if detected and detected != port:
                    self.get_logger().warning(f'Open failed on {port}, XIAO detected on {detected}, retrying…')
                else:
                    self.get_logger().warning(f'Serial open failed ({e}), retrying in 3 s…')
                time.sleep(3.0)

    def _read_serial(self):
        import time
        ser = self._open_serial()
        buf = b''
        while rclpy.ok():
            try:
                # Drain all buffered bytes at once to avoid readline() starvation under load
                waiting = ser.in_waiting
                chunk = ser.read(waiting if waiting > 0 else 1)
            except serial.SerialException as e:
                if 'returned no data' in str(e):
                    continue
                self.get_logger().warning(f'Serial read error: {e} — reopening port')
                ser.close()
                time.sleep(2.0)
                ser = self._open_serial()
                buf = b''
                continue

            if not chunk:
                continue

            buf += chunk
            *lines, buf = buf.split(b'\n')
            for raw in lines:
                line = raw.decode('ascii', errors='replace').strip()
                if not line:
                    continue
                parts = line.split(',')
                if len(parts) != _EXPECTED_FIELDS:
                    self.get_logger().warning(f'Unexpected field count ({len(parts)}): {line!r}')
                    continue
                try:
                    fields = [float(p) for p in parts]
                except ValueError:
                    self.get_logger().warning(f'Failed to parse packet: {line!r}')
                    continue
                self._pkt_queue.put(fields)

    def _publish_cb(self):
        if self._pkt_queue.empty():
            return
        # drain; use the latest packet
        fields = None
        while not self._pkt_queue.empty():
            fields = self._pkt_queue.get_nowait()
        if fields is None:
            return

        now = self.get_clock().now().to_msg()

        theta = fields[_THETA]
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = fields[_X]
        odom.pose.pose.position.y = fields[_Y]
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = fields[_VX]
        odom.twist.twist.angular.z = fields[_VTHETA]
        odom.pose.covariance = _POSE_COV
        odom.twist.covariance = _TWIST_COV
        self._odom_pub.publish(odom)

        if self._publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self._odom_frame
            t.child_frame_id = self._base_frame
            t.transform.translation.x = fields[_X]
            t.transform.translation.y = fields[_Y]
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(t)

        gx, gy, gz = fields[_GX], fields[_GY], fields[_GZ]

        if not self._calibrated:
            self._cal_buf.append((gx, gy, gz))
            remaining = _GYRO_CAL_SAMPLES - len(self._cal_buf)
            if remaining > 0 and remaining % 50 == 0:
                self.get_logger().info(
                    f'Gyro calibration: {remaining // 50} s remaining — keep robot still'
                )
            if len(self._cal_buf) >= _GYRO_CAL_SAMPLES:
                n = len(self._cal_buf)
                bx = sum(s[0] for s in self._cal_buf) / n
                by = sum(s[1] for s in self._cal_buf) / n
                bz = sum(s[2] for s in self._cal_buf) / n
                self._gyro_bias = (bx, by, bz)
                self._calibrated = True
                self._cal_buf.clear()
                self.get_logger().info(
                    f'Gyro bias calibrated — '
                    f'x:{bx:+.5f}  y:{by:+.5f}  z:{bz:+.5f} rad/s'
                )
            # Hold off on publishing /imu until calibration is done so the EKF
            # never sees the biased gyro values.
            return

        bx, by, bz = self._gyro_bias
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self._base_frame
        imu_msg.linear_acceleration.x = fields[_AX]
        imu_msg.linear_acceleration.y = fields[_AY]
        imu_msg.linear_acceleration.z = fields[_AZ]
        imu_msg.angular_velocity.x = gx - bx
        imu_msg.angular_velocity.y = gy - by
        imu_msg.angular_velocity.z = gz - bz
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        imu_msg.orientation_covariance[8] = 0.02
        imu_msg.angular_velocity_covariance[8] = 0.01
        imu_msg.linear_acceleration_covariance[0] = 0.1
        imu_msg.linear_acceleration_covariance[4] = 0.1
        imu_msg.linear_acceleration_covariance[8] = 0.1
        self._imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
