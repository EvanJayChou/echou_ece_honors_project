import math
from datetime import timedelta

import depthai as dai
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu

# Rotation taking a vector expressed in a camera optical frame (x right, y down,
# z forward) into the matching REP-103 body frame (x forward, y left, z up).
_R_LINK_FROM_OPTICAL = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])

_GYRO_COV = 0.02 ** 2    # (rad/s)^2 — post-bias-removal noise on a stationary rover
_ACCEL_COV = 0.2 ** 2    # (m/s^2)^2 — drivetrain vibration dominates


def _rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


class OakdPublisherNode(Node):
    """Publishes raw OAK-D Pro color + depth frames and the onboard IMU.

    Color and depth form a registered RGB-D pair: stereo depth is aligned on
    the device to the color camera (CAM_A), so both images share the color
    camera's intrinsics and frame_id. A Sync node pairs them by device
    timestamp, and both messages carry that capture time (not the host publish
    time), so tf lookups at the image stamp give the pose at capture. The
    color image is undistorted on the device by default, so CameraInfo
    reports zero distortion. Intrinsics come from each frame's own
    ImgTransformation, which accounts for the device-side crop and scale.

    Pose is not embedded here — downstream consumers (e.g. the keyframe bridge)
    look up the camera_frame -> map transform at the image header stamp via tf2,
    using the EKF-published odom/tf. This keeps the onboard node a plain sensor
    driver with no coupling to localization or the off-board pipeline.

    IMU samples are rotated into imu_frame, which is axis-aligned with the
    camera body link (x forward, y left, z up), so the URDF only needs the
    camera mount pose. The rotation comes from the IMU extrinsics in the device
    EEPROM, or from the manual imu_rpy parameter when those are missing.
    """

    def __init__(self):
        super().__init__('oakd_publisher_node')

        self.declare_parameter('camera_frame', 'oakd_camera_optical_frame')
        # 720p for 3DGS keyframes (the Pi 5 ran 640x480). Frame rate stays at
        # 15 Hz: keyframe_bridge only republishes on motion, and every raw frame
        # is ~2.7 MB color + 1.8 MB depth through the middleware.
        self.declare_parameter('rgb_width', 1280)
        self.declare_parameter('rgb_height', 720)
        self.declare_parameter('fps', 15.0)
        # Stereo input size. 640x400 is the OV9282 mono pair's native 2x2-binned
        # mode (full sensor is 1280x800). Depth is then aligned and scaled to
        # the color resolution on the device.
        self.declare_parameter('mono_width', 640)
        self.declare_parameter('mono_height', 400)
        # StereoDepth.PresetMode name (DEFAULT, ACCURACY, DENSITY, ROBOTICS, ...).
        self.declare_parameter('stereo_preset', 'DEFAULT')
        # Undistort color on the device so consumers (3DGS) can assume a pinhole
        # camera. If false, CameraInfo carries the EEPROM distortion instead.
        self.declare_parameter('rgb_undistort', True)
        # Manual color exposure/white balance. 0 = auto. Lock both for
        # benchmark runs so PSNR doesn't change with auto-exposure drift.
        self.declare_parameter('rgb_exposure_us', 0)
        self.declare_parameter('rgb_iso', 400)
        self.declare_parameter('rgb_white_balance_k', 0)
        # OAK-D Pro adds an active IR stereo pair (dot projector + flood light)
        # that the Lite lacks. Both default to 0 (off) — passive stereo only —
        # so behavior matches the Lite unless explicitly enabled for low-light
        # or low-texture scenes.
        self.declare_parameter('ir_dot_projector_ma', 0.0)
        self.declare_parameter('ir_flood_light_ma', 0.0)

        self.declare_parameter('enable_imu', True)
        self.declare_parameter('imu_frame', 'oakd_imu_frame')
        self.declare_parameter('imu_rate_hz', 200)
        # 'calibration' = IMU->camera rotation from the device EEPROM (falls back
        # to imu_rpy if absent); 'manual' = always use imu_rpy.
        self.declare_parameter('imu_rotation_source', 'calibration')
        self.declare_parameter('imu_rpy', [0.0, 0.0, 0.0])
        # Rover must be stationary for this long after startup. /oakd/imu is not
        # published until the gyro bias estimate is done.
        self.declare_parameter('gyro_bias_calibration_s', 5.0)

        self.camera_frame = self.get_parameter('camera_frame').value
        rgb_width = self.get_parameter('rgb_width').value
        rgb_height = self.get_parameter('rgb_height').value
        fps = self.get_parameter('fps').value
        ir_dot_projector_ma = self.get_parameter('ir_dot_projector_ma').value
        ir_flood_light_ma = self.get_parameter('ir_flood_light_ma').value
        self.enable_imu = bool(self.get_parameter('enable_imu').value)
        self.imu_frame = self.get_parameter('imu_frame').value
        imu_rate_hz = int(self.get_parameter('imu_rate_hz').value)

        mono_width = int(self.get_parameter('mono_width').value)
        mono_height = int(self.get_parameter('mono_height').value)
        stereo_preset = str(self.get_parameter('stereo_preset').value)
        self.rgb_undistort = bool(self.get_parameter('rgb_undistort').value)
        rgb_exposure_us = int(self.get_parameter('rgb_exposure_us').value)
        rgb_iso = int(self.get_parameter('rgb_iso').value)
        rgb_white_balance_k = int(self.get_parameter('rgb_white_balance_k').value)

        self.bridge = CvBridge()
        self.color_pub = self.create_publisher(Image, '/oakd/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/oakd/depth/image_raw', 10)
        self.color_info_pub = self.create_publisher(CameraInfo, '/oakd/color/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/oakd/depth/camera_info', 10)
        self._color_info = None
        self._depth_info = None

        self.device = dai.Device()
        if ir_dot_projector_ma > 0.0:
            self.device.setIrLaserDotProjectorBrightness(ir_dot_projector_ma)
        if ir_flood_light_ma > 0.0:
            self.device.setIrFloodLightBrightness(ir_flood_light_ma)

        self.pipeline = dai.Pipeline(self.device)

        # Wiring follows depthai-core v3.11.0 examples/python/ImageAlign/depth_align.py.
        cam = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        if rgb_exposure_us > 0:
            cam.initialControl.setManualExposure(rgb_exposure_us, rgb_iso)
        if rgb_white_balance_k > 0:
            cam.initialControl.setManualWhiteBalance(rgb_white_balance_k)
        rgb_out = cam.requestOutput(
            (rgb_width, rgb_height), fps=fps, enableUndistortion=self.rgb_undistort,
        )

        mono_left = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        mono_right = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        stereo = self.pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(getattr(dai.node.StereoDepth.PresetMode, stereo_preset))
        mono_left.requestOutput((mono_width, mono_height), fps=fps).link(stereo.left)
        mono_right.requestOutput((mono_width, mono_height), fps=fps).link(stereo.right)

        # Register depth to the color camera. RVC2 (OAK-D Pro) aligns inside
        # StereoDepth; RVC4 devices need a separate ImageAlign node.
        sync = self.pipeline.create(dai.node.Sync)
        sync.setSyncThreshold(timedelta(seconds=0.5 / fps))
        rgb_out.link(sync.inputs['rgb'])
        if self.device.getPlatform() == dai.Platform.RVC4:
            align = self.pipeline.create(dai.node.ImageAlign)
            stereo.depth.link(align.input)
            rgb_out.link(align.inputAlignTo)
            align.outputAligned.link(sync.inputs['depth'])
        else:
            rgb_out.link(stereo.inputAlignTo)
            stereo.depth.link(sync.inputs['depth'])
        self.rgbd_queue = sync.out.createOutputQueue(maxSize=4, blocking=False)

        self.imu_queue = None
        if self.enable_imu:
            self._setup_imu(imu_rate_hz)

        self.pipeline.start()

        # Poll at twice the frame rate; each tick drains every synced pair.
        self.timer = self.create_timer(0.5 / fps, self._publish_frames)
        if self.imu_queue is not None:
            # Each tick drains every queued packet, so one tick per sample period suffices.
            self.imu_timer = self.create_timer(1.0 / imu_rate_hz, self._publish_imu)

    def _setup_imu(self, imu_rate_hz):
        # RAW reports are the common subset of the BMI270 (current OAK-D Pro)
        # and BNO086 (early units). Neither gives a usable absolute yaw without a
        # magnetometer, so only rates/accelerations are published.
        imu = self.pipeline.create(dai.node.IMU)
        imu.enableIMUSensor(
            [dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], imu_rate_hz,
        )
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        self.imu_queue = imu.out.createOutputQueue(maxSize=50, blocking=False)
        self.imu_pub = self.create_publisher(Imu, '/oakd/imu', 50)

        self.R_frame_from_imu = self._imu_rotation()

        self._cal_samples = int(
            float(self.get_parameter('gyro_bias_calibration_s').value) * imu_rate_hz
        )
        self._cal_buf = []
        self._gyro_bias = np.zeros(3)
        self._calibrated = self._cal_samples <= 0
        if not self._calibrated:
            self.get_logger().info(
                f'Gyro bias calibration: keep the rover still for '
                f'{self._cal_samples / imu_rate_hz:.0f} s — /oakd/imu publishes after that'
            )

    def _imu_rotation(self):
        source = self.get_parameter('imu_rotation_source').value
        if source == 'calibration':
            try:
                extr = np.array(
                    self.device.readCalibration().getImuToCameraExtrinsics(
                        dai.CameraBoardSocket.CAM_A, False,
                    )
                )
                R_optical_from_imu = extr[:3, :3]
                if np.allclose(R_optical_from_imu @ R_optical_from_imu.T, np.eye(3), atol=1e-2):
                    self.get_logger().info('IMU rotation: using EEPROM IMU->CAM_A extrinsics')
                    return _R_LINK_FROM_OPTICAL @ R_optical_from_imu
                self.get_logger().warning('EEPROM IMU extrinsics are not a valid rotation')
            except Exception as exc:  # depthai raises if the EEPROM has no IMU calib
                self.get_logger().warning(f'No IMU extrinsics in EEPROM ({exc})')
            self.get_logger().warning('Falling back to the imu_rpy parameter — verify axes')

        roll, pitch, yaw = [float(v) for v in self.get_parameter('imu_rpy').value]
        return _rpy_to_matrix(roll, pitch, yaw)

    def _device_to_ros_stamp(self, device_ts) -> Time:
        # Device timestamps are host-synced steady-clock times; shift them onto the
        # ROS clock by the current steady->ROS offset.
        age_s = (dai.Clock.now() - device_ts).total_seconds()
        stamp_ns = self.get_clock().now().nanoseconds - int(age_s * 1e9)
        msg = Time()
        msg.sec = stamp_ns // 1_000_000_000
        msg.nanosec = stamp_ns % 1_000_000_000
        return msg

    def _publish_imu(self):
        for imu_data in self.imu_queue.tryGetAll():
            for packet in imu_data.packets:
                gyro = packet.gyroscope
                accel = packet.acceleroMeter
                w = self.R_frame_from_imu @ np.array([gyro.x, gyro.y, gyro.z])
                a = self.R_frame_from_imu @ np.array([accel.x, accel.y, accel.z])

                if not self._calibrated:
                    self._cal_buf.append(w)
                    if len(self._cal_buf) >= self._cal_samples:
                        self._gyro_bias = np.mean(self._cal_buf, axis=0)
                        self._calibrated = True
                        self._cal_buf = []
                        self.get_logger().info(
                            'Gyro bias calibrated: '
                            f'[{self._gyro_bias[0]:.4f}, {self._gyro_bias[1]:.4f}, '
                            f'{self._gyro_bias[2]:.4f}] rad/s'
                        )
                    continue

                w = w - self._gyro_bias
                msg = Imu()
                msg.header.stamp = self._device_to_ros_stamp(gyro.getTimestamp())
                msg.header.frame_id = self.imu_frame
                # No orientation estimate — tell consumers to ignore that field.
                msg.orientation_covariance[0] = -1.0
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (
                    float(w[0]), float(w[1]), float(w[2]))
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (
                    float(a[0]), float(a[1]), float(a[2]))
                for i in (0, 4, 8):
                    msg.angular_velocity_covariance[i] = _GYRO_COV
                    msg.linear_acceleration_covariance[i] = _ACCEL_COV
                self.imu_pub.publish(msg)

    def _camera_info(self, frame, label: str) -> CameraInfo:
        """CameraInfo from the frame's own transformation (crop/scale/undistort aware)."""
        tr = frame.getTransformation()
        width, height = tr.getSize()
        k = np.array(tr.getIntrinsicMatrix(), dtype=float)
        info = CameraInfo()
        info.width, info.height = int(width), int(height)
        info.k = k.flatten().tolist()
        info.r = np.eye(3).flatten().tolist()
        info.p = np.hstack([k, np.zeros((3, 1))]).flatten().tolist()
        if self.rgb_undistort:
            # Image was undistorted on the device; getDistortionCoefficients()
            # still reports the source sensor's, so don't copy them.
            info.distortion_model = 'plumb_bob'
            info.d = [0.0] * 5
        else:
            info.distortion_model = 'rational_polynomial'
            info.d = [float(v) for v in tr.getDistortionCoefficients()[:8]]
        self.get_logger().info(
            f'{label} intrinsics {info.width}x{info.height}: '
            f'fx={k[0, 0]:.1f} fy={k[1, 1]:.1f} cx={k[0, 2]:.1f} cy={k[1, 2]:.1f} '
            f'({info.distortion_model})'
        )
        return info

    def _publish_frames(self):
        for group in self.rgbd_queue.tryGetAll():
            rgb_frame = group['rgb']
            depth_frame = group['depth']

            if self._color_info is None:
                self._color_info = self._camera_info(rgb_frame, 'color')
                self._depth_info = self._camera_info(depth_frame, 'depth (aligned)')
                if (self._depth_info.width, self._depth_info.height) != (
                        self._color_info.width, self._color_info.height):
                    self.get_logger().warning(
                        'Aligned depth size differs from color; RGB-D pixels do not correspond 1:1'
                    )

            # One capture time for the pair: the color exposure timestamp.
            stamp = self._device_to_ros_stamp(rgb_frame.getTimestamp())

            color_msg = self.bridge.cv2_to_imgmsg(rgb_frame.getCvFrame(), encoding='bgr8')
            # REP 118: uint16 depth in millimeters is 16UC1.
            depth_msg = self.bridge.cv2_to_imgmsg(depth_frame.getCvFrame(), encoding='16UC1')
            for msg, info, img_pub, info_pub in (
                (color_msg, self._color_info, self.color_pub, self.color_info_pub),
                (depth_msg, self._depth_info, self.depth_pub, self.depth_info_pub),
            ):
                msg.header.stamp = stamp
                msg.header.frame_id = self.camera_frame
                info.header = msg.header
                img_pub.publish(msg)
                info_pub.publish(info)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OakdPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
