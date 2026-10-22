"""comms_sim: the simulated radio network and the base station.

Robots never subscribe to each other. Each robot's fleet_agent publishes its
map and status to its own outbox; this node decides who can hear whom, with
the same rule the policy was trained on (fleet_rl.comms):

    distance + wall_penalty * walls_crossed <= comm_range

using the robots' TRUE positions from Gazebo and the world's TRUE walls,
because radio physics is part of the simulated world. Every connected group
(links chain through teammates and beacons) gets its merged map and each
other's status delivered to their inboxes. The base station is a radio node
too: what it has received is the operator's map, published on /fleet/base_map.

Topics, per robot N:
    in   /robotN/ground_truth (nav_msgs/Odometry, from Gazebo)
    in   /robotN/fleet/map_out, /robotN/fleet/status_out (from fleet_agent)
    out  /robotN/fleet/map_in, /robotN/fleet/status_in (to fleet_agent)
Global:
    out  /fleet/base_map (OccupancyGrid), /fleet/coverage (Float32, 0-1),
         /fleet/markers (links, base, beacons, robots)
"""

import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy import ndimage
from std_msgs.msg import ColorRGBA, Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from fleet_rl.comms import connected_groups, link_matrix, merge_maps
from fleet_rl.config import FREE, UNKNOWN
from fleet_rl.houses import load_sdf_house

from .grid_msgs import GridGeometry, grid_to_msg, msg_to_grid

ROBOT_COLORS = [(1.0, 0.54, 0.24), (0.24, 0.77, 1.0), (0.91, 0.36, 0.77), (0.59, 0.86, 0.31),
                (1.0, 0.84, 0.25), (0.63, 0.51, 1.0)]
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CommsSim(Node):
    def __init__(self):
        super().__init__('comms_sim')
        self.robots = list(self.declare_parameter('robots', ['robot1', 'robot2', 'robot3']).value)
        world = self.declare_parameter('world', '').value
        base = (self.declare_parameter('base_x', 0.0).value,
                self.declare_parameter('base_y', 0.0).value)
        self.comm_range = float(self.declare_parameter('comm_range', 5.0).value)
        self.wall_penalty = float(self.declare_parameter('wall_penalty', 3.0).value)
        cell = float(self.declare_parameter('cell', 0.2).value)
        rate = float(self.declare_parameter('rate', 2.0).value)

        self.house = load_sdf_house(world, cell=cell, base_xy=base)
        h, w = self.house.shape
        self.geom = GridGeometry(self.house.origin[0], self.house.origin[1], cell, w, h)
        labels, _ = ndimage.label(~self.house.occupied)
        self.coverable = labels == labels[self.house.base]

        self.base_map = np.full((h, w), UNKNOWN, dtype=np.int8)
        self.true_pose = {}       # name -> (x, y, yaw, receive time)
        self.maps = {}            # name -> latest map from its outbox
        self.status = {}          # name -> latest status dict from its outbox
        self.dropped = {name: 0 for name in self.robots}
        self.beacons = []         # world (x, y) of dropped beacons
        self.inbox_map, self.inbox_status = {}, {}
        for name in self.robots:
            self.create_subscription(Odometry, f'/{name}/ground_truth',
                                     lambda m, n=name: self._on_truth(n, m), 10)
            self.create_subscription(OccupancyGrid, f'/{name}/fleet/map_out',
                                     lambda m, n=name: self._on_map(n, m), 1)
            self.create_subscription(String, f'/{name}/fleet/status_out',
                                     lambda m, n=name: self._on_status(n, m), 10)
            self.inbox_map[name] = self.create_publisher(OccupancyGrid, f'/{name}/fleet/map_in', 1)
            self.inbox_status[name] = self.create_publisher(String, f'/{name}/fleet/status_in', 10)
        self.base_pub = self.create_publisher(OccupancyGrid, '/fleet/base_map', LATCHED)
        self.coverage_pub = self.create_publisher(Float32, '/fleet/coverage', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/fleet/markers', 10)
        self.ticks = 0
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'radio: {self.comm_range:.1f} m range, {self.wall_penalty:.1f} m per wall; '
            f'robots {self.robots}; base at {base}')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_truth(self, name, msg):
        p = msg.pose.pose
        self.true_pose[name] = (p.position.x, p.position.y, _yaw(p.orientation), self._now())

    def _on_map(self, name, msg):
        try:
            self.maps[name] = msg_to_grid(msg, self.geom)
        except ValueError as e:
            self.get_logger().warn(f'{name}: {e}', throttle_duration_sec=10.0)

    def _on_status(self, name, msg):
        status = json.loads(msg.data)
        self.status[name] = status
        new = int(status.get('beacons_dropped', 0)) - self.dropped[name]
        if new > 0 and name in self.true_pose:
            x, y = self.true_pose[name][:2]
            self.beacons += [(x, y)] * new
            self.dropped[name] += new
            self.get_logger().info(f'{name} dropped a beacon at ({x:.1f}, {y:.1f})')

    def _tick(self):
        now = self._now()
        self.ticks += 1
        alive = [n for n in self.robots if n in self.true_pose and now - self.true_pose[n][3] < 2.0]
        cells = [self.geom.to_cell(*self.true_pose[n][:2]) for n in alive]
        nodes = cells + [self.house.base] + [self.geom.to_cell(x, y) for x, y in self.beacons]
        adj = link_matrix(nodes, self.house.occupied, self.comm_range, self.wall_penalty,
                          self.geom.cell)
        base_node = len(alive)
        stamp = self.get_clock().now().to_msg()

        for group in connected_groups(adj):
            members = [alive[k] for k in group if k < base_node]
            if not members:
                continue
            has_base = base_node in group
            maps = [self.maps[m] for m in members if m in self.maps]
            group_map = merge_maps(maps) if maps else None
            if has_base:
                # Newer data wins at the base, so a SLAM blip doesn't stick forever.
                if group_map is not None:
                    known = group_map != UNKNOWN
                    self.base_map[known] = group_map[known]
                group_map = self.base_map
            beacons = [list(self.beacons[k - base_node - 1]) for k in group if k > base_node]
            for m in members:
                mates = {o: {'pose': self.status[o].get('pose'), 'goal': self.status[o].get('goal'),
                             'stamp': now}
                         for o in members if o != m and o in self.status}
                status = {'connected_to_base': has_base, 'teammates': mates, 'beacons': beacons}
                if group_map is not None and (len(members) > 1 or has_base):
                    self.inbox_map[m].publish(grid_to_msg(group_map, self.geom, stamp))
                self.inbox_status[m].publish(String(data=json.dumps(status)))

        known_free = (self.base_map == FREE) & self.coverable
        self.coverage_pub.publish(Float32(data=float(known_free.sum() / self.coverable.sum())))
        if self.ticks % 2 == 0:
            self.base_pub.publish(grid_to_msg(self.base_map, self.geom, stamp))
        self.marker_pub.publish(self._markers(alive, nodes, adj, stamp))

    def _markers(self, alive, nodes, adj, stamp):
        out = MarkerArray()

        def marker(ns, mid, kind, rgba, scale):
            m = Marker()
            m.header.frame_id, m.header.stamp = 'world', stamp
            m.ns, m.id, m.type, m.action = ns, mid, kind, Marker.ADD
            m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
            m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
            m.pose.orientation.w = 1.0
            m.lifetime.sec = 2
            return m

        links = marker('links', 0, Marker.LINE_LIST, (0.30, 0.76, 0.54, 1.0), (0.05, 0, 0))
        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                if adj[i, j]:
                    for k in (i, j):
                        x, y = self.geom.to_world(nodes[k])
                        links.points.append(Point(x=x, y=y, z=0.6))
        out.markers.append(links)

        bx, by = self.geom.to_world(self.house.base)
        base = marker('base', 0, Marker.CUBE, (1.0, 1.0, 1.0, 1.0), (0.5, 0.5, 0.5))
        base.pose.position.x, base.pose.position.y, base.pose.position.z = bx, by, 0.25
        out.markers.append(base)

        for k, (x, y) in enumerate(self.beacons):
            b = marker('beacons', k, Marker.CYLINDER, (0.95, 0.79, 0.30, 1.0), (0.25, 0.25, 0.5))
            b.pose.position.x, b.pose.position.y, b.pose.position.z = x, y, 0.25
            out.markers.append(b)

        for name in alive:
            i = self.robots.index(name)
            x, y, yaw = self.true_pose[name][:3]
            label = marker('names', i, Marker.TEXT_VIEW_FACING, (1.0, 1.0, 1.0, 1.0), (0, 0, 0.35))
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, 1.0
            label.text = name
            out.markers.append(label)
            body = marker('robots', i, Marker.CUBE, (*ROBOT_COLORS[i % 6], 1.0), (0.8, 0.76, 0.3))
            body.pose.position.x, body.pose.position.y, body.pose.position.z = x, y, 0.3
            body.pose.orientation.z, body.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            out.markers.append(body)
        return out


def main():
    rclpy.init()
    node = CommsSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
