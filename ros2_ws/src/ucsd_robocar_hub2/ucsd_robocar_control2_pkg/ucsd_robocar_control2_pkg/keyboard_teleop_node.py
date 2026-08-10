import sys
import tty
import termios
import select
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

NODE_NAME = 'keyboard_teleop_node'

BANNER = """
--- Keyboard Teleop (Ackermann) ---
  w : forward          s : backward
  a : steer left       d : steer right
  space : stop (zero speed + center steering)
  r / f : increase / decrease speed step
  e / c : increase / decrease steer step
  q : quit
"""

SPEED_STEP = 0.05   # m/s
STEER_STEP = 0.05   # radians
MAX_SPEED = 0.5     # m/s
MAX_STEER = 0.4     # radians (~23 deg)


def get_key(timeout=0.1):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if ready else ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
        self.speed_step = 0.05   # m/s initial increment
        self.steer_step = 0.10   # rad initial increment
        self.speed = 0.0
        self.steer = 0.0

    def _publish(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steer
        self.pub.publish(msg)

    def _stop(self):
        self.speed = 0.0
        self.steer = 0.0
        self._publish()

    def run(self):
        print(BANNER)
        print(f'speed_step={self.speed_step:.2f} m/s  steer_step={self.steer_step:.2f} rad')
        try:
            while rclpy.ok():
                key = get_key()
                if not key:
                    continue

                if key == 'w':
                    self.speed = min(self.speed + self.speed_step, MAX_SPEED)
                elif key == 's':
                    self.speed = max(self.speed - self.speed_step, -MAX_SPEED)
                elif key == 'a':
                    self.steer = min(self.steer + self.steer_step, MAX_STEER)
                elif key == 'd':
                    self.steer = max(self.steer - self.steer_step, -MAX_STEER)
                elif key == ' ':
                    self._stop()
                    continue
                elif key == 'r':
                    self.speed_step = min(self.speed_step + SPEED_STEP, MAX_SPEED)
                    print(f'speed_step={self.speed_step:.2f} m/s')
                elif key == 'f':
                    self.speed_step = max(self.speed_step - SPEED_STEP, SPEED_STEP)
                    print(f'speed_step={self.speed_step:.2f} m/s')
                elif key == 'e':
                    self.steer_step = min(self.steer_step + STEER_STEP, MAX_STEER)
                    print(f'steer_step={self.steer_step:.2f} rad')
                elif key == 'c':
                    self.steer_step = max(self.steer_step - STEER_STEP, STEER_STEP)
                    print(f'steer_step={self.steer_step:.2f} rad')
                elif key in ('q', '\x03'):
                    break

                print(f'speed={self.speed:+.2f} m/s  steer={self.steer:+.3f} rad')
                self._publish()
        finally:
            self._stop()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
