"""fleet_agent: one robot's decision maker in Gazebo (runs in the robot's namespace).

Every decision_period (3 s, as in training):
  1. belief = my SLAM map, resampled onto the world grid, merged with the
     last map comms_sim delivered from teammates / the base
  2. build exactly the observation used in training (fleet_rl.observations)
  3. the policy picks an option: a frontier, a teammate, the base, stay, or
     drop a beacon
  4. frontier / teammate / base -> a Nav2 NavigateToPose goal; stay -> cancel
     the goal; drop -> tell comms_sim (via my status) and stay
Nav2 plans and drives, and lidar_safety_node still gates every velocity
command, so the policy can't steer the robot into anything.

This node never sees ground truth: only its own SLAM map and TF, plus what
comms_sim chooses to deliver.

Robots and each other: the rover's body is lower than the lidar plane, so
fleet robots carry a mast their teammates' lidars can see (rover.urdf.xacro).
As real fleets do, robots also share positions over the radio: this node
publishes the teammates it currently hears from as a small point cloud
(fleet/teammates) that Nav2's local costmap uses as a second obstacle source.
Since SLAM maps a teammate's mast like any other obstacle, anywhere a robot has
stood is treated as floor: each agent keeps a trail of its own and its
teammates' positions (plus the base) and marks it free in its belief.
"""

import json
import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA, Header, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from fleet_rl.baselines import Breadcrumb, Greedy, ReturnToBase
from fleet_rl.comms import merge_maps
from fleet_rl.config import FREE, SimConfig
from fleet_rl.observations import (SLOT_BASE, SLOT_TEAMMATE, AgentView, Teammate,
                                   build_observation)
from fleet_rl.policy import DEFAULT_MODEL, LearnedPolicy

from .comms_sim import ROBOT_COLORS
from .grid_msgs import GridGeometry, grid_to_msg, msg_to_grid, slam_to_world
from .progress import ProgressWatchdog

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
GOAL_CHANGE_M = 0.75   # re-send a Nav2 goal only if the target moved this much
TRAIL_RADIUS_M = 0.5   # cells this close to where a robot stood are floor (the rover is 0.8 m long)
ARRIVED_M = 1.0        # a "succeeded" goal farther away than this gets sent again
TEAMMATE_FRESH_S = 2.0  # only teammates heard from this recently go into Nav2's costmaps
STUCK_S, STUCK_M = 15.0, 0.3  # give up on a goal after moving under 0.3 m in 15 s (sim time)
# A teammate as obstacle points: its center plus two rings (the rover is 0.80 x 0.76 m).
_RING = [(r * math.cos(a), r * math.sin(a)) for r in (0.25, 0.45)
         for a in np.linspace(0, 2 * math.pi, 16, endpoint=False)] + [(0.0, 0.0)]


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class FleetAgent(Node):
    def __init__(self):
        super().__init__('fleet_agent')
        p = self.declare_parameter
        self.robots = list(p('robots', ['robot1', 'robot2', 'robot3']).value)
        self.name = self.get_namespace().strip('/')
        if self.name not in self.robots:
            raise RuntimeError(f'namespace "{self.name}" is not in robots={self.robots}')
        self.my_id = self.robots.index(self.name)
        policy = p('policy', 'learned').value
        model = p('model', '').value or DEFAULT_MODEL
        period = float(p('return_period', 60.0).value)
        self.cfg = SimConfig(comm_range=float(p('comm_range', 5.0).value),
                             wall_penalty=float(p('wall_penalty', 3.0).value),
                             cell=float(p('cell', 0.2).value))
        self.geom = GridGeometry(float(p('grid_origin_x', 0.0).value),
                                 float(p('grid_origin_y', 0.0).value), self.cfg.cell,
                                 int(p('grid_width', 1).value), int(p('grid_height', 1).value))
        base_x, base_y = float(p('base_x', 0.0).value), float(p('base_y', 0.0).value)
        self.base_cell = self.geom.to_cell(base_x, base_y)
        # Where this robot parks when it goes to the base: its own spawn spot (known
        # to fit, 1.3 m from the others, in radio contact). Grid-sim robots pass
        # through each other, so the policy's one shared point 1 m from the base
        # made Gazebo robots crowd there and pin each other against the furniture.
        self.home = (float(p('home_x', base_x).value), float(p('home_y', base_y).value))
        self.mission_seconds = float(p('mission_seconds', 240.0).value)
        self.beacons_left = int(p('beacons', 0).value)
        self.escape_radius = float(p('escape_radius', 0.6).value)
        self.world_frame = p('world_frame', 'world').value
        self.map_frame, self.base_frame = f'{self.name}/map', f'{self.name}/body_link'

        self.policy = {'learned': lambda: LearnedPolicy(model),
                       'greedy': Greedy,
                       'return': lambda: ReturnToBase(period_s=period),
                       'breadcrumb': lambda: Breadcrumb(period_s=period)}[policy]()

        shape = (self.geom.height, self.geom.width)
        self.slam_msg = None
        self.own = None                       # my SLAM map on the world grid
        self.inbox = None                     # last merged map from comms_sim
        self.reported = np.zeros(shape, dtype=bool)
        self.trail = np.zeros(shape, dtype=bool)  # where robots have stood: floor for sure
        k = math.ceil(TRAIL_RADIUS_M / self.cfg.cell)
        dr, dc = np.mgrid[-k:k + 1, -k:k + 1]
        self.trail_offsets = np.argwhere(np.hypot(dr, dc) * self.cfg.cell <= TRAIL_RADIUS_M) - k
        self._mark_trail(*self.geom.to_world(self.base_cell))
        self.connected = False
        self.last_base_contact = None
        self.teammates = {}                   # name -> (pose, goal, stamp)
        self.beacons = set()                  # world (x, y)
        self.beacons_dropped = 0
        self.goal = None                      # dict(xy, cell, kind, id, handle, state)
        self.goal_counter = 0
        self.blacklist = []                   # frontier cells we failed to reach
        self.watchdog = ProgressWatchdog(STUCK_S, STUCK_M)
        self.start_time = None
        self.last_decision = None
        self.map_pose = None                  # (x, y, yaw) of my map frame in the world
        self.pose = None                      # my (x, y) in the world

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(OccupancyGrid, 'map', self._on_slam_map, LATCHED)
        self.create_subscription(OccupancyGrid, 'fleet/map_in', self._on_map_in, 1)
        self.create_subscription(String, 'fleet/status_in', self._on_status_in, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, 'fleet/map_out', LATCHED)
        self.status_pub = self.create_publisher(String, 'fleet/status_out', 10)
        self.decision_pub = self.create_publisher(String, 'fleet/decision', 10)
        self.target_pub = self.create_publisher(Marker, 'fleet/target', 10)
        self.mates_pub = self.create_publisher(PointCloud2, 'fleet/teammates', 10)
        self.create_timer(float(p('decision_period', 3.0).value), self._decide)
        self.create_timer(1.0, self._publish_map)
        self.create_timer(0.5, self._publish_status)
        self.create_timer(0.5, self._publish_teammates)
        self.get_logger().info(f'{self.name}: policy={policy}, id {self.my_id}, '
                               f'radio {self.cfg.comm_range:.1f} m')

    # ------------------------------------------------------------ inputs

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _lookup(self, child):
        t = self.tf.lookup_transform(self.world_frame, child, Time()).transform
        return t.translation.x, t.translation.y, _yaw(t.rotation)

    def _on_slam_map(self, msg):
        self.slam_msg = msg
        if self.map_pose is None:
            try:
                self.map_pose = self._lookup(self.map_frame)
            except TransformException:
                return
        self.own = slam_to_world(msg, self.map_pose, self.geom)

    def _on_map_in(self, msg):
        self.inbox = msg_to_grid(msg, self.geom)

    def _on_status_in(self, msg):
        status = json.loads(msg.data)
        self.connected = bool(status['connected_to_base'])
        if self.connected:
            self.last_base_contact = self._now()
            if self.inbox is not None:      # the inbox map is the base's map right now
                self.reported = self.inbox == FREE
        for name, info in status['teammates'].items():
            if info.get('pose') is not None:
                self.teammates[name] = (info['pose'], info.get('goal'), info['stamp'])
                self._mark_trail(*info['pose'])
        self.beacons |= {tuple(b) for b in status['beacons']}

    def _mark_trail(self, x, y):
        cells = self.trail_offsets + self.geom.to_cell(x, y)
        ok = ((cells >= 0) & (cells < (self.geom.height, self.geom.width))).all(axis=1)
        self.trail[cells[ok, 0], cells[ok, 1]] = True

    def _belief(self):
        maps = [m for m in (self.own, self.inbox) if m is not None]
        belief = merge_maps(maps) if len(maps) > 1 else maps[0].copy()
        belief[self.trail] = FREE
        return belief

    # ------------------------------------------------------------ outputs

    def _publish_map(self):
        if self.own is not None:
            self.map_pub.publish(grid_to_msg(self._belief(), self.geom,
                                             self.get_clock().now().to_msg()))

    def _publish_status(self):
        try:
            self.pose = self._lookup(self.base_frame)[:2]
        except TransformException:
            return
        self._mark_trail(*self.pose)
        status = {'pose': list(self.pose), 'goal': list(self.goal['xy']) if self.goal else None,
                  'beacons_dropped': self.beacons_dropped}
        self.status_pub.publish(String(data=json.dumps(status)))

    def _publish_teammates(self):
        """Teammates I heard from recently, as points for Nav2's local costmap (world frame)."""
        now = self._now()
        points = [(x + dx, y + dy, 0.3)
                  for (x, y), _, stamp in self.teammates.values() if now - stamp < TEAMMATE_FRESH_S
                  for dx, dy in _RING]
        header = Header(frame_id=self.world_frame, stamp=self.get_clock().now().to_msg())
        self.mates_pub.publish(point_cloud2.create_cloud_xyz32(header, points))

    # ------------------------------------------------------------ deciding

    def _decide(self):
        try:
            x, y, _ = self._lookup(self.base_frame)
        except TransformException as e:
            self.get_logger().info(f'waiting for TF: {e}', throttle_duration_sec=10.0)
            return
        self.pose = (x, y)
        if self.own is None or not self.nav.server_is_ready():
            self.get_logger().info('waiting for my SLAM map and Nav2...',
                                   throttle_duration_sec=10.0)
            return
        now = self._now()
        if self.start_time is None:
            self.start_time = now
            self.last_base_contact = self.last_base_contact or now
            self.get_logger().info('exploring')
        time_left = max(0.0, 1.0 - (now - self.start_time) / self.mission_seconds)
        if (self.goal is not None and self.goal['state'] == 'active'
                and self.watchdog.stuck(self.pose, now)):
            # No cancel here: the next goal replaces this one in Nav2, and cancelling
            # right before sending it can cancel the new goal too. _cancel_goal stops
            # it if the policy chooses to stay instead.
            self._give_up(f'stuck for {STUCK_S:.0f} s')

        belief = self._belief()
        mates = [Teammate(id=self.robots.index(n), pos=self.geom.to_cell(*pose),
                          goal=self.geom.to_cell(*goal) if goal else None, age=now - stamp)
                 for n, (pose, goal, stamp) in self.teammates.items() if n in self.robots]
        view = AgentView(
            belief=belief, pos=self.geom.to_cell(x, y), base=self.base_cell, my_id=self.my_id,
            teammates=mates, beacons=[self.geom.to_cell(*b) for b in sorted(self.beacons)],
            beacons_left=self.beacons_left,
            unreported_cells=int(((belief == FREE) & ~self.reported).sum()),
            since_base_contact=now - self.last_base_contact, connected_to_base=self.connected,
            time_left=time_left, fleet_size=len(self.robots),
            current_target=self.goal['cell'] if self.goal else None, blacklist=self.blacklist)
        obs = build_observation(view, self.cfg, escape_radius_m=self.escape_radius)
        if time_left > 0.0:
            kind, target = obs.targets[self.policy.act(obs)]
        else:   # out of time: drive home (1 m short of the base, like training) and stay
            kind, target = obs.targets[SLOT_BASE] if obs.mask[SLOT_BASE] else ('stay', None)
            if target == view.pos:
                kind = 'stay'

        if kind in ('frontier', 'teammate', 'base'):
            xy = self.home if kind == 'base' else self.geom.to_world(target)
            if (self.goal is None or self.goal['state'] == 'failed'
                    or math.dist(xy, self.goal['xy']) > GOAL_CHANGE_M
                    or (self.goal['state'] == 'arrived'
                        and math.dist(self.pose, self.goal['xy']) > ARRIVED_M)):
                # (Nav2 can report success early: its stateful goal checker keeps
                # "already there" across a preempted goal. Re-send if we aren't.)
                self._send_goal(xy, target, kind)
        elif kind == 'drop' and self.beacons_left > 0:
            self.beacons_left -= 1
            self.beacons_dropped += 1
            self._cancel_goal()
        else:
            self._cancel_goal()
        where = f' ({self.goal["xy"][0]:.1f}, {self.goal["xy"][1]:.1f})' if self.goal else ''
        prefix = '' if time_left > 0.0 else 'mission over: '
        decision = f'{prefix}{kind}{where}'
        self.decision_pub.publish(String(data=decision))
        if decision != self.last_decision:   # log changes only, so the terminal stays readable
            self.last_decision = decision
            self.get_logger().info(
                f'{decision}  [{int(obs.mask[:SLOT_TEAMMATE].sum())} frontiers, '
                f'base {"in" if self.connected else "out of"} range, '
                f'{view.unreported_cells * self.cfg.cell ** 2:.0f} m² unreported]')

    def _send_goal(self, xy, cell, kind):
        px, py, pyaw = self.map_pose
        dx, dy = xy[0] - px, xy[1] - py
        heading = math.atan2(xy[1] - self.pose[1], xy[0] - self.pose[0]) - pyaw
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.pose.position.x = math.cos(pyaw) * dx + math.sin(pyaw) * dy
        goal.pose.pose.position.y = -math.sin(pyaw) * dx + math.cos(pyaw) * dy
        goal.pose.pose.orientation.z = math.sin(heading / 2)
        goal.pose.pose.orientation.w = math.cos(heading / 2)

        self.goal_counter += 1
        # Like the grid sim, the goal stays this robot's "current target" after it
        # arrives (state 'arrived'), until the policy picks something else.
        self.goal = dict(xy=xy, cell=cell, kind=kind, id=self.goal_counter, handle=None,
                         state='active')
        self.watchdog.reset(self.pose, self._now())
        future = self.nav.send_goal_async(goal)
        future.add_done_callback(lambda f, gid=self.goal_counter: self._on_goal_response(f, gid))
        self._show_target(xy)

    def _on_goal_response(self, future, gid):
        handle = future.result()
        if self.goal is None or self.goal['id'] != gid:
            if handle.accepted and self.goal is None:
                handle.cancel_goal_async()   # we chose to stay before Nav2 even answered
            return
        if not handle.accepted:
            self.goal['state'] = 'failed'
            return
        self.goal['handle'] = handle
        handle.get_result_async().add_done_callback(lambda f: self._on_result(f, gid))

    def _on_result(self, future, gid):
        if self.goal is None or self.goal['id'] != gid or self.goal['state'] != 'active':
            return   # an old goal we already replaced, or one the watchdog gave up on
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.goal['state'] = 'arrived'
            return
        self._give_up('Nav2 could not reach it')

    def _give_up(self, why):
        """Mark the goal failed; a frontier we couldn't reach is never offered again."""
        self.goal['state'] = 'failed'
        if self.goal['kind'] == 'frontier':
            self.blacklist.append(self.goal['cell'])
        x, y = self.goal['xy']
        self.get_logger().info(f'gave up on {self.goal["kind"]} ({x:.1f}, {y:.1f}): {why}')

    def _cancel_goal(self):
        # 'failed' too: a goal the watchdog gave up on may still be running in Nav2
        # (cancelling one Nav2 already finished is harmless).
        if self.goal is not None and self.goal['state'] != 'arrived' and self.goal['handle']:
            self.goal['handle'].cancel_goal_async()
        self.goal = None

    def _show_target(self, xy):
        m = Marker()
        m.header.frame_id = self.world_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'target', self.my_id, Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = xy[0], xy[1], 0.2
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.35
        r, g, b = ROBOT_COLORS[self.my_id % len(ROBOT_COLORS)]
        m.color = ColorRGBA(r=r, g=g, b=b, a=0.9)
        self.target_pub.publish(m)


def main():
    rclpy.init()
    node = FleetAgent()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
