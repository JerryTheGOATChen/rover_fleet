"""Map conversions and per-robot parameter rewriting."""

import math
import os

import numpy as np
import yaml
from nav_msgs.msg import OccupancyGrid

from fleet_rl.config import FREE, OCCUPIED, UNKNOWN
from rover_fleet.grid_msgs import GridGeometry, grid_to_msg, msg_to_grid, slam_to_world
from rover_fleet.launch_utils import namespaced_params, rewrite_for_robot

GEOM = GridGeometry(origin_x=-2.0, origin_y=-1.0, cell=0.2, width=30, height=20)
CONFIG = os.path.join(os.path.dirname(__file__), '..', '..', 'rover_bringup', 'config')


def test_grid_message_round_trip():
    grid = np.random.default_rng(0).choice([UNKNOWN, FREE, OCCUPIED], size=(20, 30)).astype(np.int8)
    msg = grid_to_msg(grid, GEOM, stamp=OccupancyGrid().header.stamp)
    assert msg.header.frame_id == 'world' and msg.info.width == 30
    assert (msg_to_grid(msg, GEOM) == grid).all()


def test_geometry_cells():
    assert GEOM.to_cell(-2.0, -1.0) == (0, 0)
    assert GEOM.to_cell(-1.9, -0.7) == (1, 0)
    assert GEOM.to_world((0, 0)) == (-1.9, -0.9)
    assert GEOM.to_cell(100.0, 100.0) == (19, 29)       # clipped onto the grid


def slam_map(res=0.05, w=40, h=20):
    """A SLAM-style map: all free, with a wall along its column 20 (x = 1.0..1.05 m)."""
    msg = OccupancyGrid()
    msg.info.resolution, msg.info.width, msg.info.height = res, w, h
    msg.info.origin.orientation.w = 1.0
    data = np.zeros((h, w), dtype=np.int8)
    data[:, 20] = 100
    data[:, 35:] = -1
    msg.data = data.ravel().tolist()
    return msg


def test_slam_map_lands_in_the_right_world_cells():
    # The SLAM map frame sits at world (1.0, 0.0), not rotated.
    grid = slam_to_world(slam_map(), (1.0, 0.0, 0.0), GEOM)
    wall_col = GEOM.to_cell(1.0 + 1.0 + 0.025, 0.5)[1]   # map x 1.0 m -> world x 2.0 m
    rows = slice(GEOM.to_cell(0, 0.05)[0], GEOM.to_cell(0, 0.95)[0] + 1)
    assert (grid[rows, wall_col] == OCCUPIED).all()
    assert (grid[rows, GEOM.to_cell(1.5, 0.5)[1]] == FREE).all()
    assert (grid[rows, GEOM.to_cell(2.9, 0.5)[1]] == UNKNOWN).all()   # map x > 1.75: unknown
    assert (grid[:GEOM.to_cell(0, -0.1)[0] + 1] == UNKNOWN).all()      # below the map


def test_slam_map_rotation():
    # Rotated 90 degrees: the wall (map x = 1 m) becomes a row at world y = 1 m.
    grid = slam_to_world(slam_map(), (0.0, 0.0, math.pi / 2), GEOM)
    row = GEOM.to_cell(0.0, 1.0 + 0.025)[0]
    cols = slice(GEOM.to_cell(-0.95, 0)[1], GEOM.to_cell(-0.05, 0)[1] + 1)
    assert (grid[row, cols] == OCCUPIED).all()


def test_rewrite_prefixes_frames_and_absolute_topics_only():
    tree = {'bt_navigator': {'ros__parameters': {
        'global_frame': 'map', 'robot_base_frame': 'body_link', 'odom_topic': '/odom',
        'local_costmap_topic': 'local_costmap/costmap_raw', 'mode': 'mapping',
        'observation': {'topic': '/scan'}, 'plugins': ['map', 'inflation']}}}
    out = rewrite_for_robot(tree, 'robot2')['bt_navigator']['ros__parameters']
    assert out['global_frame'] == 'robot2/map'
    assert out['robot_base_frame'] == 'robot2/body_link'
    assert out['odom_topic'] == '/robot2/odom'
    assert out['observation']['topic'] == '/robot2/scan'
    assert out['local_costmap_topic'] == 'local_costmap/costmap_raw'   # relative: untouched
    assert out['mode'] == 'mapping'


def test_real_config_files_rewrite_cleanly():
    nav = yaml.safe_load(open(namespaced_params(os.path.join(CONFIG, 'nav2.yaml'), 'robot1')))
    assert list(nav) == ['robot1']
    nav = nav['robot1']
    assert nav['bt_navigator']['ros__parameters']['global_frame'] == 'robot1/map'
    local = nav['local_costmap']['local_costmap']['ros__parameters']
    assert local['global_frame'] == 'robot1/odom'
    assert local['obstacle_layer']['scan']['topic'] == '/robot1/scan'
    assert nav['global_costmap']['global_costmap']['ros__parameters']['static_layer'][
        'map_topic'] == '/robot1/map'
    slam = yaml.safe_load(open(namespaced_params(os.path.join(CONFIG, 'slam.yaml'), 'robot1')))
    params = slam['robot1']['slam_toolbox']['ros__parameters']
    assert (params['map_frame'], params['odom_frame'], params['base_frame']) == \
        ('robot1/map', 'robot1/odom', 'robot1/body_link')
    assert params['scan_topic'] == '/robot1/scan' and params['map_name'] == '/robot1/map'
    assert params['ceres_loss_function'] == 'None'          # strings survive the round trip


def test_overrides_merge_into_the_robot_params():
    extra = {'bt_navigator': {'ros__parameters': {'default_nav_to_pose_bt_xml': '/tmp/tree.xml'}}}
    nav = yaml.safe_load(open(namespaced_params(os.path.join(CONFIG, 'nav2.yaml'), 'robot3',
                                                extra)))['robot3']
    params = nav['bt_navigator']['ros__parameters']
    assert params['default_nav_to_pose_bt_xml'] == '/tmp/tree.xml'
    assert params['global_frame'] == 'robot3/map'           # the rest is still there


def test_repeated_overrides_are_written_without_yaml_aliases():
    # The same dict used twice must not become an &anchor / *alias: rcl can't parse those.
    layer = {'obstacle_layer': {'observation_sources': 'scan teammates'}}
    out = namespaced_params(os.path.join(CONFIG, 'nav2.yaml'), 'robot1', {
        'local_costmap': {'local_costmap': {'ros__parameters': layer}},
        'global_costmap': {'global_costmap': {'ros__parameters': layer}}})
    text = open(out).read()
    assert '&id' not in text and '*id' not in text


def test_ros_can_parse_every_generated_params_file():
    """The real check: rcl itself loads the files (it rejects things PyYAML accepts)."""
    import rclpy
    layer = {'obstacle_layer': {'observation_sources': 'scan teammates',
                                'teammates': {'topic': '/robot1/fleet/teammates'}}}
    extra = {'local_costmap': {'local_costmap': {'ros__parameters': layer}},
             'global_costmap': {'global_costmap': {'ros__parameters': layer}}}
    for name, overrides in (('nav2.yaml', extra), ('slam.yaml', None), ('twist_mux.yaml', None)):
        path = namespaced_params(os.path.join(CONFIG, name), 'robot1', overrides)
        context = rclpy.context.Context()
        rclpy.init(args=['--ros-args', '--params-file', path], context=context)
        rclpy.shutdown(context=context)
