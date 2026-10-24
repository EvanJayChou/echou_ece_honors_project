"""Logitech F710 gamepad teleop -> /ackermann_cmd.

Button/axis roles follow DonkeyCar's LogitechJoystickController:
  left stick horizontal  : steering
  right stick vertical   : throttle (scaled by speed_scale * max_speed)
  A                      : emergency stop: zero speed, cruise off, and held until
                           the throttle stick is back at center
  Back                   : toggle constant throttle (cruise at speed_scale * max_speed)
  D-pad up / down        : increase / decrease speed_scale
  LB (optional)          : deadman, only when deadman_button >= 0

Indices default to the SDL GameController layout published by the `joy`
package's game_controller_node, with the F710's mode switch on X (XInput).

Commands are published at publish_rate while the sticks are deflected or
cruise is on. Once everything is back at neutral, one zero command is sent and
the node goes quiet, so it doesn't fight Nav2 for /ackermann_cmd. If /joy stops
arriving (receiver unplugged, controller out of range or asleep), the car is
stopped.
"""
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

NODE_NAME = 'f710_teleop_node'


class F710Teleop(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # Limits (match keyboard_teleop_node)
        self.max_speed = self.declare_parameter('max_speed', 0.5).value         # m/s
        self.max_steer = self.declare_parameter('max_steer', 0.4).value         # rad
        self.speed_scale = self.declare_parameter('speed_scale', 0.5).value     # 0..1
        self.speed_scale_step = self.declare_parameter('speed_scale_step', 0.05).value
        self.min_speed_scale = self.declare_parameter('min_speed_scale', 0.05).value

        # Mapping (SDL GameController layout from game_controller_node)
        self.steer_axis = self.declare_parameter('steer_axis', 0).value          # LEFTX
        self.throttle_axis = self.declare_parameter('throttle_axis', 3).value    # RIGHTY
        self.invert_steer = self.declare_parameter('invert_steer', False).value
        self.invert_throttle = self.declare_parameter('invert_throttle', False).value
        self.estop_button = self.declare_parameter('estop_button', 0).value      # A
        self.cruise_button = self.declare_parameter('cruise_button', 4).value    # BACK
        self.scale_up_button = self.declare_parameter('scale_up_button', 11).value      # DPAD_UP
        self.scale_down_button = self.declare_parameter('scale_down_button', 12).value  # DPAD_DOWN
        self.deadman_button = self.declare_parameter('deadman_button', -1).value  # -1 = off; 9 = LB

        self.deadzone = self.declare_parameter('deadzone', 0.05).value
        self.joy_timeout = self.declare_parameter('joy_timeout', 0.5).value      # s
        publish_rate = self.declare_parameter('publish_rate', 20.0).value       # Hz

        self.pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
        self.create_subscription(Joy, '/joy', self._on_joy, 10)
        self.create_timer(1.0 / publish_rate, self._on_timer)

        self.steer_in = 0.0
        self.throttle_in = 0.0
        self.deadman_held = self.deadman_button < 0
        self.cruise = False
        self.estopped = False
        self.prev_buttons = []
        self.last_joy_time = None
        self.active = False   # True while we're the ones commanding the car

        self.get_logger().info(
            f'F710 teleop ready: max_speed={self.max_speed:.2f} m/s, '
            f'speed_scale={self.speed_scale:.2f}, max_steer={self.max_steer:.2f} rad')

    # ── input ────────────────────────────────────────────────────────────────
    def _axis(self, msg, idx, invert):
        if idx < 0 or idx >= len(msg.axes):
            return 0.0
        v = msg.axes[idx]
        if abs(v) < self.deadzone:
            return 0.0
        return -v if invert else v

    def _pressed(self, msg, idx):
        """Rising edge on button idx."""
        if idx < 0 or idx >= len(msg.buttons):
            return False
        prev = self.prev_buttons[idx] if idx < len(self.prev_buttons) else 0
        return msg.buttons[idx] == 1 and prev == 0

    def _on_joy(self, msg):
        if self.last_joy_time is None:
            self.get_logger().info('Gamepad connected (/joy receiving)')
        self.last_joy_time = self.get_clock().now()

        self.steer_in = self._axis(msg, self.steer_axis, self.invert_steer)
        self.throttle_in = self._axis(msg, self.throttle_axis, self.invert_throttle)
        if self.deadman_button >= 0:
            self.deadman_held = (self.deadman_button < len(msg.buttons)
                                 and msg.buttons[self.deadman_button] == 1)

        if self._pressed(msg, self.estop_button):
            self.estopped = True
            self.cruise = False
            self.get_logger().warn('E-STOP — center the throttle stick to re-arm')
        elif self.estopped and self.throttle_in == 0.0:
            self.estopped = False
            self.get_logger().info('E-stop cleared')

        if self._pressed(msg, self.cruise_button) and not self.estopped:
            self.cruise = not self.cruise
            self.get_logger().info(
                f'Constant throttle {"ON" if self.cruise else "OFF"} '
                f'({self.speed_scale * self.max_speed:.2f} m/s)')

        old_scale = self.speed_scale
        if self._pressed(msg, self.scale_up_button):
            self.speed_scale = round(min(1.0, self.speed_scale + self.speed_scale_step), 2)
        if self._pressed(msg, self.scale_down_button):
            self.speed_scale = round(
                max(self.min_speed_scale, self.speed_scale - self.speed_scale_step), 2)
        if self.speed_scale != old_scale:
            self._log_scale()

        self.prev_buttons = list(msg.buttons)

    def _log_scale(self):
        self.get_logger().info(
            f'speed_scale={self.speed_scale:.2f}  '
            f'(top speed {self.speed_scale * self.max_speed:.2f} m/s)')

    # ── output ───────────────────────────────────────────────────────────────
    def _on_timer(self):
        if self.last_joy_time is None:
            return
        age = (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9
        if age > self.joy_timeout:
            if self.active or self.cruise:
                self.get_logger().warn(
                    f'No /joy for {age:.1f} s — stopping (controller off or out of range?)')
            self.cruise = False
            self.steer_in = self.throttle_in = 0.0
            self.last_joy_time = None
            self._stop_if_active()
            return

        if self.estopped or not self.deadman_held:
            self._stop_if_active()
            return

        top = self.speed_scale * self.max_speed
        speed = top if self.cruise else self.throttle_in * top
        steer = self.steer_in * self.max_steer

        if speed == 0.0 and steer == 0.0:
            self._stop_if_active()
            return

        self.active = True
        self._publish(speed, steer)

    def _stop_if_active(self):
        if self.active:
            self._publish(0.0, 0.0)
            self.active = False

    def _publish(self, speed, steer):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.pub.publish(msg)


def main(args=None):
    # Keep the context alive on Ctrl+C so the final stop command still goes out.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = F710Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
