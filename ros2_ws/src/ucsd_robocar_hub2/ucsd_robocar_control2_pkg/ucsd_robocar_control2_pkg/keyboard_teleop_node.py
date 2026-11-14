"""Keyboard teleop -> /ackermann_cmd, with the same feel as f710_teleop_node
(DonkeyCar's LogitechJoystickController).

Throttle latches and steering springs back:
  w / s  (or Up / Down)    : throttle. w drives forward at speed_scale * max_speed
                             and keeps going without holding it; s does the
                             same in reverse. Pressing the opposite key while
                             moving stops (one press back to 0 m/s).
  a / d  (or Left / Right) : steer left / right at steer_scale * max_steer while
                             held; release and the wheels re-center. Steering
                             never changes the throttle.
  space                    : stop: zero speed and center the wheels
  r / f                    : increase / decrease speed_scale (applies at once)
  t / g                    : increase / decrease steer_scale
  q / Ctrl+C               : quit (sends a final stop)

The throttle latches because a terminal only reports key presses, never
releases, and only auto-repeats the most recently pressed key: while you hold
w and tap a, w stops repeating even though it's still down, so a hold-to-drive
throttle would cut out whenever you steer. Steering is "held" while its key
keeps auto-repeating: initial_hold seconds after the first press (must cover the
terminal's auto-repeat delay), then release_timeout seconds after the last
repeat. If the terminal or SSH session goes away, the node sends a stop.

Commands are published at publish_rate while moving. Once both channels are
back at neutral, one zero command is sent and the node goes quiet, so it
doesn't fight Nav2 for /ackermann_cmd.
"""
import os
import signal
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from ackermann_msgs.msg import AckermannDriveStamped

NODE_NAME = 'keyboard_teleop_node'

# Escape sequences for the arrow keys -> equivalent letter key
ARROWS = {'\x1b[A': 'w', '\x1b[B': 's', '\x1b[C': 'd', '\x1b[D': 'a',
          '\x1bOA': 'w', '\x1bOB': 's', '\x1bOC': 'd', '\x1bOD': 'a'}


class HeldKey:
    """Tracks whether a direction key is being held, from auto-repeat timing."""

    def __init__(self):
        self.direction = 0      # -1, 0, +1
        self.pressed_at = 0.0   # first press of the current hold
        self.last_seen = 0.0    # latest press or repeat

    def press(self, direction, now):
        if direction != self.direction:
            self.direction = direction
            self.pressed_at = now
        self.last_seen = now

    def update(self, now, initial_hold, release_timeout):
        if not self.direction:
            return 0
        repeating = self.last_seen > self.pressed_at
        limit = release_timeout if repeating else initial_hold
        if now - self.last_seen > limit:
            self.direction = 0
        return self.direction

    def clear(self):
        self.direction = 0


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # Limits (match f710_teleop_node)
        self.max_speed = self.declare_parameter('max_speed', 0.5).value         # m/s
        self.max_steer = self.declare_parameter('max_steer', 0.4).value         # rad
        self.speed_scale = self.declare_parameter('speed_scale', 0.5).value     # 0..1
        self.speed_scale_step = self.declare_parameter('speed_scale_step', 0.05).value
        self.min_speed_scale = self.declare_parameter('min_speed_scale', 0.05).value
        self.steer_scale = self.declare_parameter('steer_scale', 0.75).value    # 0..1
        self.steer_scale_step = self.declare_parameter('steer_scale_step', 0.125).value

        # Key-hold detection (see module docstring)
        self.initial_hold = self.declare_parameter('initial_hold', 0.7).value        # s
        self.release_timeout = self.declare_parameter('release_timeout', 0.25).value  # s
        self.publish_rate = self.declare_parameter('publish_rate', 20.0).value       # Hz

        self.pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)

        self.throttle = 0     # -1 reverse, 0 stopped, +1 forward (latched)
        self.steer = HeldKey()
        self.active = False   # True while we're the ones commanding the car
        self.last_status = None

    # ── input ────────────────────────────────────────────────────────────────
    def _keys(self, data):
        """Split raw terminal input into keys, mapping arrows to w/a/s/d."""
        i = 0
        while i < len(data):
            seq = data[i:i + 3]
            if seq in ARROWS:
                yield ARROWS[seq]
                i += 3
            else:
                yield data[i]
                i += 1

    def _handle(self, key, now):
        """Returns False to quit."""
        if key == 'w':
            self.throttle = 0 if self.throttle < 0 else +1
        elif key == 's':
            self.throttle = 0 if self.throttle > 0 else -1
        elif key == 'a':
            self.steer.press(+1, now)          # REP-103: positive = left
        elif key == 'd':
            self.steer.press(-1, now)
        elif key == ' ':
            self.throttle = 0
            self.steer.clear()
        elif key == 'r':
            self.speed_scale = round(min(1.0, self.speed_scale + self.speed_scale_step), 3)
            self._say_scales()
        elif key == 'f':
            self.speed_scale = round(
                max(self.min_speed_scale, self.speed_scale - self.speed_scale_step), 3)
            self._say_scales()
        elif key == 't':
            self.steer_scale = round(min(1.0, self.steer_scale + self.steer_scale_step), 3)
            self._say_scales()
        elif key == 'g':
            self.steer_scale = round(
                max(self.steer_scale_step, self.steer_scale - self.steer_scale_step), 3)
            self._say_scales()
        elif key in ('q', '\x03', '\x04'):
            return False
        return True

    # ── output ───────────────────────────────────────────────────────────────
    def _say(self, text):
        print(text, flush=True)

    def _say_scales(self):
        self._say(f'top speed {self.speed_scale * self.max_speed:.2f} m/s, '
                  f'steer {self.steer_scale * self.max_steer:.2f} rad')

    def _command(self, now):
        steer = self.steer.update(now, self.initial_hold, self.release_timeout)
        speed = self.throttle * self.speed_scale * self.max_speed
        return speed, steer * self.steer_scale * self.max_steer

    def _publish(self, speed, steer):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.pub.publish(msg)

    def _output(self, speed, steer):
        if speed == 0.0 and steer == 0.0:
            if self.active:
                self._publish(0.0, 0.0)
                self.active = False
        else:
            self.active = True
            self._publish(speed, steer)

        status = (round(speed, 3), round(steer, 3))
        if status != self.last_status:
            self.last_status = status
            self._say(f'speed={speed:+.2f} m/s  steer={steer:+.3f} rad')

    def run(self):
        if not sys.stdin.isatty():
            self.get_logger().error('keyboard_teleop_node needs an interactive terminal')
            return
        self._say(__doc__.split('\n\n')[1])
        self._say_scales()

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        period = 1.0 / self.publish_rate
        # SSH/terminal hangup: stop the car instead of dying with it moving.
        signal.signal(signal.SIGHUP, _raise_interrupt)
        try:
            tty.setcbreak(fd)   # unbuffered, no echo; Ctrl+C still interrupts
            next_tick = time.monotonic()
            while rclpy.ok():
                timeout = max(0.0, next_tick - time.monotonic())
                ready, _, _ = select.select([fd], [], [], timeout)
                now = time.monotonic()
                if ready:
                    try:
                        data = os.read(fd, 64).decode(errors='ignore')
                    except OSError:
                        break   # terminal gone
                    if not data:
                        break   # stdin closed
                    if not all(self._handle(k, now) for k in self._keys(data)):
                        break
                if now >= next_tick:
                    self._output(*self._command(now))
                    next_tick = now + period
        except KeyboardInterrupt:
            pass
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            except termios.error:
                pass
            self._publish(0.0, 0.0)


def _raise_interrupt(signum, frame):
    raise KeyboardInterrupt


def main(args=None):
    # Keep the context alive on Ctrl+C so the final stop command still goes out.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = KeyboardTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
