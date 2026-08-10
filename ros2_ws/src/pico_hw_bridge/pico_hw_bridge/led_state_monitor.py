import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ackermann_msgs.msg import AckermannDriveStamped

# m/s — anything above this is "moving"
_SPEED_THRESHOLD = 0.01
# seconds without a /ackermann_cmd message before returning to idle
_CMD_TIMEOUT_SEC = 1.5


class LedStateMonitor(Node):
    """
    Translates rover motion state to Pico LED strip commands via pico_hw_server.

    States:
      idle    → LED:SUCCESS (green)  — no motion command or speed ≈ 0
      moving  → LED:UNKNOWN (blue)   — active drive command with |speed| > threshold

    Shutdown (FAILURE/OFF) is handled by the calling shell script, not here.
    """

    def __init__(self):
        super().__init__('led_state_monitor')

        self._state: str | None = None
        self._last_cmd_time = None

        self._cli_idle   = self.create_client(Trigger, 'pico/led_success')
        self._cli_moving = self.create_client(Trigger, 'pico/led_unknown')

        self.create_subscription(
            AckermannDriveStamped,
            '/ackermann_cmd',
            self._ackermann_cb,
            10,
        )

        # Tick every second: sets initial idle state and catches command timeout
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            'LED state monitor active — idle→SUCCESS  moving→UNKNOWN'
        )

    def _ackermann_cb(self, msg: AckermannDriveStamped) -> None:
        self._last_cmd_time = self.get_clock().now()
        moving = abs(msg.drive.speed) > _SPEED_THRESHOLD
        self._transition('moving' if moving else 'idle')

    def _tick(self) -> None:
        if self._last_cmd_time is None:
            self._transition('idle')
            return
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > _CMD_TIMEOUT_SEC:
            self._transition('idle')

    def _transition(self, new_state: str) -> None:
        if new_state == self._state:
            return
        self._state = new_state
        if new_state == 'idle':
            self.get_logger().info('LED → SUCCESS (idle)')
            self._call(self._cli_idle)
        else:
            self.get_logger().info('LED → UNKNOWN (moving / exploring)')
            self._call(self._cli_moving)

    def _call(self, client: rclpy.client.Client) -> None:
        if not client.service_is_ready():
            self.get_logger().warn(
                'pico_hw_server not ready — LED update skipped'
            )
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(self._on_done)

    def _on_done(self, future) -> None:
        try:
            result = future.result()
            if not result.success:
                self.get_logger().warn(f'LED command nacked: {result.message}')
        except Exception as exc:
            self.get_logger().warn(f'LED service error: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = LedStateMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
