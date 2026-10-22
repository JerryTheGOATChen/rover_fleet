"""record_run: save a Gazebo fleet run as a top-down GIF + coverage CSV.

    ros2 run rover_fleet record_run --ros-args -p use_sim_time:=true \\
        -p duration:=240.0 -p out:=/tmp/fleet_run

Draws the same picture as the grid simulator (fleet_rl.render): what the base
knows, what each robot knows but hasn't reported, robots, radio links and
beacons. Writes <out>.gif, <out>.png (last frame) and <out>.csv.
"""

import csv
import os

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

from fleet_rl.config import FREE, UNKNOWN
from fleet_rl.houses import GAZEBO_BASE, load_sdf_house
from fleet_rl.render import draw_frame, save_gif

from .grid_msgs import GridGeometry, msg_to_grid

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Recorder(Node):
    def __init__(self):
        super().__init__('record_run')
        self.robots = list(self.declare_parameter('robots', ['robot1', 'robot2', 'robot3']).value)
        world = self.declare_parameter('world', '').value or os.path.join(
            get_package_share_directory('rover_bringup'), 'worlds', 'rover_world.sdf')
        self.duration = float(self.declare_parameter('duration', 240.0).value)
        self.out = os.path.expanduser(self.declare_parameter('out', '/tmp/fleet_run').value)
        self.title = self.declare_parameter('title', '').value

        self.house = load_sdf_house(world, base_xy=GAZEBO_BASE)
        h, w = self.house.shape
        self.geom = GridGeometry(self.house.origin[0], self.house.origin[1], self.house.cell, w, h)
        self.base_map = np.full((h, w), UNKNOWN, dtype=np.int8)
        self.robot_maps = {n: np.full((h, w), UNKNOWN, dtype=np.int8) for n in self.robots}
        self.pose, self.target = {}, {}
        self.links, self.beacons, self.coverage = [], [], 0.0

        self.create_subscription(OccupancyGrid, '/fleet/base_map', self._on_base, LATCHED)
        self.create_subscription(Float32, '/fleet/coverage', self._on_coverage, 10)
        self.create_subscription(MarkerArray, '/fleet/markers', self._on_markers, 10)
        for i, n in enumerate(self.robots):
            self.create_subscription(OccupancyGrid, f'/{n}/fleet/map_out',
                                     lambda m, n=n: self._on_robot_map(n, m), LATCHED)
            self.create_subscription(Odometry, f'/{n}/ground_truth',
                                     lambda m, n=n: self._on_pose(n, m), 10)
            self.create_subscription(Marker, f'/{n}/fleet/target',
                                     lambda m, n=n: self._on_target(n, m), 10)
        self.frames, self.rows, self.t0 = [], [], None
        self.create_timer(1.0, self._capture)
        self.get_logger().info(f'recording {self.duration:.0f} s of sim time to {self.out}.gif')

    def _cell(self, x, y):
        return self.geom.to_cell(x, y)

    def _on_base(self, msg):
        self.base_map = msg_to_grid(msg, self.geom)

    def _on_coverage(self, msg):
        self.coverage = msg.data

    def _on_robot_map(self, name, msg):
        self.robot_maps[name] = msg_to_grid(msg, self.geom)

    def _on_pose(self, name, msg):
        p = msg.pose.pose.position
        self.pose[name] = self._cell(p.x, p.y)

    def _on_target(self, name, msg):
        self.target[name] = self._cell(msg.pose.position.x, msg.pose.position.y)

    def _on_markers(self, msg):
        links, beacons = [], []
        for m in msg.markers:
            if m.ns == 'links':
                pts = [self._cell(p.x, p.y) for p in m.points]
                links = list(zip(pts[0::2], pts[1::2]))
            elif m.ns == 'beacons':
                beacons.append(self._cell(m.pose.position.x, m.pose.position.y))
        self.links, self.beacons = links, beacons

    def _capture(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if not self.pose:
            return
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        nodes = sorted({p for link in self.links for p in link})
        adj = np.zeros((len(nodes), len(nodes)), dtype=bool)
        for a, b in self.links:
            adj[nodes.index(a), nodes.index(b)] = adj[nodes.index(b), nodes.index(a)] = True
        self.frames.append({
            't': t, 'coverage': self.coverage, 'base_belief': self.base_map.copy(),
            'robot_free': [self.robot_maps[n] == FREE for n in self.robots],
            'robots': [(self.pose[n], True, self.target.get(n)) for n in self.robots
                       if n in self.pose],
            'nodes': nodes, 'links': adj, 'beacons': list(self.beacons)})
        self.rows.append((round(t, 1), round(self.coverage, 4)))
        if t >= self.duration:
            self.save()
            raise SystemExit

    def save(self):
        if not self.frames:
            return
        save_gif(self.house, self.frames, self.out + '.gif', scale=7, title=self.title,
                 ms_per_frame=80)
        draw_frame(self.house, self.frames[-1], scale=7, title=self.title).save(self.out + '.png')
        with open(self.out + '.csv', 'w', newline='') as f:
            csv.writer(f).writerows([('t', 'base_coverage')] + self.rows)
        self.get_logger().info(f'saved {len(self.frames)} frames to {self.out}.gif/.png/.csv')


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save()
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
