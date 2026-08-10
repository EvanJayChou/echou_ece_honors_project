import math
from collections import deque
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


# action_msgs/GoalStatus values used by Nav2 goal result status
STATUS_SUCCEEDED = 4
STATUS_ABORTED = 6
STATUS_CANCELED = 5


@dataclass
class FrontierCandidate:
    wx: float
    wy: float
    gain: float


class FrontierExplorerNode(Node):
    def __init__(self):
        super().__init__('frontier_explorer_node')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('mission_control_topic', '/mission/control')
        self.declare_parameter('state_topic', '/mission/state')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')

        self.declare_parameter('auto_start', False)
        self.declare_parameter('publish_end_on_complete', True)

        self.declare_parameter('free_threshold', 20)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('min_frontier_cluster_size', 8)
        self.declare_parameter('blacklist_radius_m', 0.6)
        self.declare_parameter('blacklist_timeout_s', 45.0)
        self.declare_parameter('replan_period_s', 2.0)
        self.declare_parameter('goal_timeout_s', 45.0)
        self.declare_parameter('no_frontier_cycles_before_complete', 5)
        self.declare_parameter('min_goal_separation_m', 0.7)
        self.declare_parameter('distance_weight', 1.0)
        self.declare_parameter('gain_weight', 2.0)

        self.map_topic = self.get_parameter('map_topic').value
        self.mission_control_topic = self.get_parameter('mission_control_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value

        self.auto_start = bool(self.get_parameter('auto_start').value)
        self.publish_end_on_complete = bool(self.get_parameter('publish_end_on_complete').value)

        self.free_threshold = int(self.get_parameter('free_threshold').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.min_frontier_cluster_size = int(self.get_parameter('min_frontier_cluster_size').value)
        self.blacklist_radius_m = float(self.get_parameter('blacklist_radius_m').value)
        self.blacklist_timeout_s = float(self.get_parameter('blacklist_timeout_s').value)
        self.replan_period_s = float(self.get_parameter('replan_period_s').value)
        self.goal_timeout_s = float(self.get_parameter('goal_timeout_s').value)
        self.no_frontier_cycles_before_complete = int(self.get_parameter('no_frontier_cycles_before_complete').value)
        self.min_goal_separation_m = float(self.get_parameter('min_goal_separation_m').value)
        self.distance_weight = float(self.get_parameter('distance_weight').value)
        self.gain_weight = float(self.get_parameter('gain_weight').value)

        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, 10)
        self.cmd_sub = self.create_subscription(String, self.mission_control_topic, self._mission_control_cb, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.cmd_pub = self.create_publisher(String, self.mission_control_topic, 10)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.map_msg: Optional[OccupancyGrid] = None
        self.exploration_active = self.auto_start
        self.no_frontier_cycles = 0

        self.active_goal_handle = None
        self.active_goal_future = None
        self.active_goal_result_future = None
        self.active_goal_sent_time = None
        self.active_goal_xy: Optional[Tuple[float, float]] = None
        self.last_goal_xy: Optional[Tuple[float, float]] = None

        self.blacklist: List[Tuple[float, float, float]] = []

        self.timer = self.create_timer(self.replan_period_s, self._tick)

        self.get_logger().info(
            f'frontier_explorer started (auto_start={self.auto_start}, publish_end_on_complete={self.publish_end_on_complete})'
        )

    def _publish_state(self, text: str):
        msg = String()
        msg.data = text
        self.state_pub.publish(msg)

    def _publish_control(self, command: str):
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)

    def _map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg

    def _mission_control_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'start':
            self.exploration_active = True
            self.no_frontier_cycles = 0
            self._publish_state('exploration_started')
            self.get_logger().info('Exploration enabled by mission start command')
            return

        if cmd == 'end':
            self.exploration_active = False
            self.no_frontier_cycles = 0
            self._cancel_active_goal('mission end command received')
            self._publish_state('exploration_stopped')
            self.get_logger().info('Exploration disabled by mission end command')

    def _tick(self):
        self._prune_blacklist()

        if not self.exploration_active:
            return

        if self.map_msg is None:
            self.get_logger().debug('Waiting for map before exploration can begin')
            return

        robot_xy = self._lookup_robot_xy()
        if robot_xy is None:
            self.get_logger().warning('No TF map->base available yet; exploration paused')
            return

        if self.active_goal_handle is not None:
            if self.active_goal_sent_time is not None:
                elapsed = (self.get_clock().now() - self.active_goal_sent_time).nanoseconds / 1e9
                if elapsed > self.goal_timeout_s:
                    self._blacklist_active_goal('goal timed out')
                    self._cancel_active_goal('goal timeout')
            return

        candidates = self._compute_frontier_candidates(self.map_msg)
        if not candidates:
            self.no_frontier_cycles += 1
            self.get_logger().info(
                f'No usable frontiers ({self.no_frontier_cycles}/{self.no_frontier_cycles_before_complete})'
            )
            if self.no_frontier_cycles >= self.no_frontier_cycles_before_complete:
                self._publish_state('exploration_complete')
                self.exploration_active = False
                self.get_logger().info('Exploration appears complete (no frontiers found repeatedly)')
                if self.publish_end_on_complete:
                    self._publish_control('end')
                    self.get_logger().info('Published mission end command after exploration completion')
            return

        best = self._select_best_candidate(candidates, robot_xy)
        if best is None:
            self.no_frontier_cycles += 1
            return

        self.no_frontier_cycles = 0
        self._send_nav_goal(best)

    def _lookup_robot_xy(self) -> Optional[Tuple[float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            return (float(tf.transform.translation.x), float(tf.transform.translation.y))
        except TransformException:
            return None

    def _compute_frontier_candidates(self, map_msg: OccupancyGrid) -> List[FrontierCandidate]:
        width = map_msg.info.width
        height = map_msg.info.height
        if width == 0 or height == 0:
            return []

        data = list(map_msg.data)

        frontier_flags = [False] * (width * height)
        for y in range(height):
            for x in range(width):
                idx = y * width + x
                value = data[idx]
                if value < 0 or value > self.free_threshold:
                    continue
                if self._has_unknown_neighbor(data, width, height, x, y):
                    frontier_flags[idx] = True

        clusters: List[List[Tuple[int, int]]] = []
        visited = [False] * (width * height)

        for y in range(height):
            for x in range(width):
                idx = y * width + x
                if not frontier_flags[idx] or visited[idx]:
                    continue

                cluster = []
                q = deque([(x, y)])
                visited[idx] = True

                while q:
                    cx, cy = q.popleft()
                    cluster.append((cx, cy))
                    for nx, ny in self._neighbors8(cx, cy, width, height):
                        nidx = ny * width + nx
                        if frontier_flags[nidx] and not visited[nidx]:
                            visited[nidx] = True
                            q.append((nx, ny))

                if len(cluster) >= self.min_frontier_cluster_size:
                    clusters.append(cluster)

        candidates: List[FrontierCandidate] = []
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        resolution = map_msg.info.resolution

        for cluster in clusters:
            cx = sum(p[0] for p in cluster) / len(cluster)
            cy = sum(p[1] for p in cluster) / len(cluster)
            wx = origin_x + (cx + 0.5) * resolution
            wy = origin_y + (cy + 0.5) * resolution
            gain = len(cluster) * resolution

            if self._is_blacklisted(wx, wy):
                continue
            if self.last_goal_xy is not None:
                if self._distance((wx, wy), self.last_goal_xy) < self.min_goal_separation_m:
                    continue

            candidates.append(FrontierCandidate(wx=wx, wy=wy, gain=gain))

        return candidates

    def _has_unknown_neighbor(self, data: List[int], width: int, height: int, x: int, y: int) -> bool:
        for nx, ny in self._neighbors4(x, y, width, height):
            nval = data[ny * width + nx]
            if nval < 0:
                return True
        return False

    def _select_best_candidate(
        self, candidates: List[FrontierCandidate], robot_xy: Tuple[float, float]
    ) -> Optional[FrontierCandidate]:
        best = None
        best_score = None

        for c in candidates:
            dist = self._distance((c.wx, c.wy), robot_xy)
            score = self.distance_weight * dist - self.gain_weight * c.gain
            if best is None or score < best_score:
                best = c
                best_score = score

        return best

    def _send_nav_goal(self, c: FrontierCandidate):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning('navigate_to_pose action server unavailable')
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = c.wx
        goal.pose.pose.position.y = c.wy
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        self.active_goal_future = self.nav_client.send_goal_async(goal)
        self.active_goal_future.add_done_callback(self._goal_response_cb)
        self.active_goal_sent_time = self.get_clock().now()
        self.active_goal_xy = (c.wx, c.wy)
        self.last_goal_xy = (c.wx, c.wy)

        self._publish_state('exploration_goal_sent')
        self.get_logger().info(f'Sent frontier goal x={c.wx:.2f}, y={c.wy:.2f}, gain={c.gain:.2f}')

    def _goal_response_cb(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to send goal: {exc}')
            self.active_goal_future = None
            self.active_goal_sent_time = None
            self.active_goal_xy = None
            return

        if not goal_handle.accepted:
            self.get_logger().warning('Frontier goal rejected by Nav2')
            self.active_goal_future = None
            self.active_goal_sent_time = None
            self._blacklist_active_goal('goal rejected')
            self.active_goal_xy = None
            return

        self.active_goal_handle = goal_handle
        self.active_goal_result_future = goal_handle.get_result_async()
        self.active_goal_result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        status = None
        try:
            result_wrap = future.result()
            status = result_wrap.status
        except Exception as exc:
            self.get_logger().error(f'Goal result callback failed: {exc}')

        if status == STATUS_SUCCEEDED:
            self.get_logger().info('Frontier goal succeeded')
            self._publish_state('exploration_goal_reached')
        elif status in (STATUS_ABORTED, STATUS_CANCELED):
            self.get_logger().warning(f'Frontier goal finished with status {status}; blacklisting point')
            self._blacklist_active_goal(f'goal status {status}')
            self._publish_state('exploration_goal_failed')
        else:
            self.get_logger().warning(f'Frontier goal finished with status {status}')
            self._blacklist_active_goal(f'goal status {status}')

        self.active_goal_handle = None
        self.active_goal_future = None
        self.active_goal_result_future = None
        self.active_goal_sent_time = None
        self.active_goal_xy = None

    def _cancel_active_goal(self, reason: str):
        if self.active_goal_handle is None:
            return

        self.get_logger().info(f'Canceling active goal: {reason}')
        cancel_future = self.active_goal_handle.cancel_goal_async()

        def _cancel_done(_future):
            self.active_goal_handle = None
            self.active_goal_future = None
            self.active_goal_result_future = None
            self.active_goal_sent_time = None
            self.active_goal_xy = None

        cancel_future.add_done_callback(_cancel_done)

    def _blacklist_active_goal(self, reason: str):
        if self.active_goal_xy is None:
            return
        expires = (self.get_clock().now().nanoseconds / 1e9) + self.blacklist_timeout_s
        self.blacklist.append((self.active_goal_xy[0], self.active_goal_xy[1], expires))
        self.get_logger().info(
            f'Blacklisted goal x={self.active_goal_xy[0]:.2f}, y={self.active_goal_xy[1]:.2f} ({reason})'
        )

    def _prune_blacklist(self):
        now_s = self.get_clock().now().nanoseconds / 1e9
        self.blacklist = [entry for entry in self.blacklist if entry[2] > now_s]

    def _is_blacklisted(self, x: float, y: float) -> bool:
        for bx, by, _ in self.blacklist:
            if self._distance((x, y), (bx, by)) <= self.blacklist_radius_m:
                return True
        return False

    def _neighbors4(self, x: int, y: int, width: int, height: int):
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx = x + dx
            ny = y + dy
            if 0 <= nx < width and 0 <= ny < height:
                yield nx, ny

    def _neighbors8(self, x: int, y: int, width: int, height: int):
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx = x + dx
                ny = y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    yield nx, ny

    def _distance(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
