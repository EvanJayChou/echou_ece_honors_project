import depthai as dai
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class OakdPublisherNode(Node):
    """Publishes raw OAK-D Pro color + depth frames at sensor rate.

    Pose is not embedded here — downstream consumers (e.g. the keyframe bridge)
    look up the camera_frame -> map transform at the image header stamp via tf2,
    using the EKF-published odom/tf. This keeps the onboard node a plain sensor
    driver with no coupling to localization or the off-board pipeline.
    """

    def __init__(self):
        super().__init__('oakd_publisher_node')

        self.declare_parameter('camera_frame', 'oakd_camera_optical_frame')
        self.declare_parameter('rgb_width', 640)
        self.declare_parameter('rgb_height', 480)
        self.declare_parameter('fps', 15.0)
        # OAK-D Pro adds an active IR stereo pair (dot projector + flood light)
        # that the Lite lacks. Both default to 0 (off) — passive stereo only —
        # so behavior matches the Lite unless explicitly enabled for low-light
        # or low-texture scenes.
        self.declare_parameter('ir_dot_projector_ma', 0.0)
        self.declare_parameter('ir_flood_light_ma', 0.0)

        self.camera_frame = self.get_parameter('camera_frame').value
        rgb_width = self.get_parameter('rgb_width').value
        rgb_height = self.get_parameter('rgb_height').value
        fps = self.get_parameter('fps').value
        ir_dot_projector_ma = self.get_parameter('ir_dot_projector_ma').value
        ir_flood_light_ma = self.get_parameter('ir_flood_light_ma').value

        self.bridge = CvBridge()
        self.color_pub = self.create_publisher(Image, '/oakd/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/oakd/depth/image_raw', 10)

        self.device = dai.Device()
        if ir_dot_projector_ma > 0.0:
            self.device.setIrLaserDotProjectorBrightness(ir_dot_projector_ma)
        if ir_flood_light_ma > 0.0:
            self.device.setIrFloodLightBrightness(ir_flood_light_ma)

        self.pipeline = dai.Pipeline(self.device)

        cam = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        rgb_cap = dai.ImgFrameCapability()
        rgb_cap.size.fixed((rgb_width, rgb_height))
        rgb_cap.fps.fixed(fps)
        self.rgb_queue = cam.requestOutput(rgb_cap).createOutputQueue()

        mono_left = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        mono_right = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        stereo = self.pipeline.create(dai.node.StereoDepth)
        mono_left.requestOutput((rgb_width, rgb_height)).link(stereo.left)
        mono_right.requestOutput((rgb_width, rgb_height)).link(stereo.right)
        self.depth_queue = stereo.depth.createOutputQueue()

        self.pipeline.start()

        self.timer = self.create_timer(1.0 / fps, self._publish_frames)

    def _publish_frames(self):
        now = self.get_clock().now().to_msg()

        rgb_frame = self.rgb_queue.tryGet()
        if rgb_frame is not None:
            msg = self.bridge.cv2_to_imgmsg(rgb_frame.getCvFrame(), encoding='bgr8')
            msg.header.stamp = now
            msg.header.frame_id = self.camera_frame
            self.color_pub.publish(msg)

        depth_frame = self.depth_queue.tryGet()
        if depth_frame is not None:
            msg = self.bridge.cv2_to_imgmsg(depth_frame.getCvFrame(), encoding='mono16')
            msg.header.stamp = now
            msg.header.frame_id = self.camera_frame
            self.depth_pub.publish(msg)

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
